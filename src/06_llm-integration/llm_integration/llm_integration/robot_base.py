"""robot_base — the shared ROS node that drives the LLM agent.

It is robot-agnostic: a concrete robot (micro / stretch) subclasses RobotInterface
and fills in four hooks:

    describe()    -> str            natural-language robot + environment description
    tools()       -> list[schema]   the movement tools the LLM may call
    state_json()  -> dict           current robot state (small, JSON-able)
    dispatch(name, args) -> dict    actually move the robot, return a result

The base class owns the one-shot command interface: it subscribes to
``/llm/command`` (std_msgs/String). Each message published there is treated as one
instruction — the agent runs once, the robot moves, and we wait for the next.

Concurrency note: the agent loop blocks (it waits on the model and on motion). We
run under a MultiThreadedExecutor with a ReentrantCallbackGroup so that, while the
command callback is blocked inside a motion, the executor's other threads keep
servicing action-client futures and the state subscriptions. That is what lets the
Stretch FollowJointTrajectory calls complete without a re-entrant-spin error.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from llm_integration.ollama_agent import run_agent
from llm_integration.prompt import build_system_prompt, build_user_message

COMMAND_TOPIC = '/llm/command'
RESPONSE_TOPIC = '/llm/response'

# A "no-action" tool offered to every robot: the model calls it to ANSWER a
# question (or otherwise reply) without moving. Handled centrally in
# RobotInterface, so the per-robot adapters only declare movement tools.
RESPOND_TOOL = {
    'type': 'function',
    'function': {
        'name': 'respond',
        'description': ('Answer the user in natural language WITHOUT moving the '
                        'robot. Use for questions about the robot or its state, '
                        'or whenever no motion is needed.'),
        'parameters': {
            'type': 'object',
            'properties': {
                'answer': {'type': 'string',
                           'description': 'the natural-language reply to show the user'}},
            'required': ['answer']},
    },
}


def spin_wait(future, timeout=30.0):
    """Block until an async future completes, while the MT executor services it."""
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline and rclpy.ok():
        time.sleep(0.02)
    return future.result() if future.done() else None


class RobotInterface(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.cb_group = ReentrantCallbackGroup()

        # qwen3:1.7b is the default: fast tool-calling on CPU (~2-3 s warm).
        # qwen3:4b is more accurate on directions but much slower; pass model:= to
        # switch to it (or any other Ollama tool-calling model).
        self.declare_parameter('model', 'qwen3:1.7b')
        self.declare_parameter('backend', 'ollama')   # 'ollama' | 'mock'
        self.declare_parameter('ollama_host', '')      # '' -> default localhost
        # Thinking models (qwen3) are far too slow on CPU with reasoning ON, so we
        # default it OFF for fast, direct tool calls. Set think:=true to re-enable.
        self.declare_parameter('think', False)
        self.model = self.get_parameter('model').value
        self.backend = self.get_parameter('backend').value
        self.host = self.get_parameter('ollama_host').value
        self.think = self.get_parameter('think').value

        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10,
                                 callback_group=self.cb_group)
        # The model's natural-language reply about what it did (or its answer to a
        # question) is published here so a UI/terminal can show it as the response.
        self.response_pub = self.create_publisher(String, RESPONSE_TOPIC, 10)
        self.get_logger().info(
            f"LLM motion controller up (model={self.model}, backend={self.backend}). "
            f"Publish an instruction:\n"
            f"  ros2 topic pub --once {COMMAND_TOPIC} std_msgs/String "
            f"\"data: 'turn left and drive forward a bit'\"")

    # ---- hooks the concrete robot must implement ---------------------------
    def describe(self):
        raise NotImplementedError

    def tools(self):
        raise NotImplementedError

    def state_json(self):
        raise NotImplementedError

    def dispatch(self, name, args):
        raise NotImplementedError

    # ---- one-shot command handling -----------------------------------------
    def _on_command(self, msg):
        instruction = msg.data.strip()
        if not instruction:
            return
        self.get_logger().info(f'>>> instruction: {instruction!r}')
        state = self.state_json()
        self.get_logger().info(f'    state: {state}')
        try:
            reply = run_agent(
                model=self.model,
                system_prompt=build_system_prompt(self.describe()),
                user_message=build_user_message(instruction, state),
                tools=self.tools() + [RESPOND_TOOL],
                dispatch=self._dispatch,
                backend=self.backend,
                host=self.host,
                think=self.think,
                logger=lambda m: self.get_logger().info(f'    {m}'),
            )
        except Exception as exc:
            self.get_logger().error(f'agent failed: {exc}')
            reply = f'(agent error: {exc})'
        self.get_logger().info(f'<<< reply: {reply}')
        self.response_pub.publish(String(data=reply))

    def _dispatch(self, name, args):
        # 'respond' is the no-action tool: just carry the model's answer back.
        if name in ('respond', 'no_action'):
            answer = args.get('answer') or args.get('message') or ''
            result = {'ok': True, 'action': 'no_action', 'answer': answer}
        else:
            result = self.dispatch(name, args)
        self.get_logger().info(f'    -> {result}')
        return result


def run_node(interface_cls):
    """Generic entry point: spin one robot interface under a MT executor."""
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = interface_cls()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
