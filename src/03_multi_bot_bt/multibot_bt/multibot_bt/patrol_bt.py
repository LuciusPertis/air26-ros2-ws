"""patrol_bt — the per-unit Behaviour Tree patrol brain (py_trees).

One tree, two formations, with recovery. Ticked at 10 Hz:

    Selector "patrol"
    |- Sequence "safety"   : FrontBlocked?  -> Avoid
    |- Sequence "follow"   : HaveReference? -> HoldFormation     # I can see my ref -> hold
    |- Sequence "recover"  : ShouldFollow?  -> SearchAndRecover  # lost it -> 360 search
    `- Lead                                                      # genuinely the leader -> patrol

Recovery (vision-only): a unit that should be following but can't see its reference spins
360 to reacquire ANY peer marker or the world anchor; the moment a reference reappears the
higher-priority "follow" branch takes over. An expanding search creeps forward between spins;
after a timeout it gives up and leads.

Leader vs lost (anchor/pose-based): leadership is decided from the world anchor, not just
"I see no one". Each unit publishes its distance to the world anchor (relative_localizer ->
anchor_range); the unit nearest the anchor is "frontmost" = leader. So a lost follower (not
frontmost) searches instead of wandering, and after a column turnaround the new front unit's
search reacquires the anchor, finds it is frontmost, and auto-promotes to leader.

  switch live: ros2 service call /<ns>/set_formation multibot_interfaces/srv/SetFormation "{formation: parallel}"
"""

import math
import random
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray

from multibot_interfaces.srv import SetFormation

S = py_trees.common.Status
UNITS = ['r1', 'r2', 'r3']


# ----------------------------- helpers on ctx -----------------------------
def have_reference(c):
    """Do I currently have contact with my formation reference?"""
    if c.formation == 'convoy':
        return (c.now() - c.back_t) < c.p.marker_timeout
    # parallel: the right marker recently, OR the side US sees a neighbour at a sane gap
    gap = c.left if c.p.gap_side == 'left' else c.right
    return (c.now() - c.right_t) < c.p.marker_timeout or (0.2 < gap < 1.5)


def frontmost(c):
    """By anchor distance, am I the unit nearest the world anchor (=patrol front)?"""
    mine = c.anchor_range.get(c.me)
    if mine is None or (c.now() - mine[1]) > c.p.anchor_fresh or mine[0] < 0:
        return False
    best = mine[0]
    for u in UNITS:
        r = c.anchor_range.get(u)
        if u != c.me and r and (c.now() - r[1]) < c.p.anchor_fresh and 0 <= r[0] < best:
            return False
    return True


def is_leader_role(c):
    if c.formation == 'parallel':
        return c.me == c.p.leader_ns
    # convoy: leader if I've not been a follower recently, or the anchor says I'm frontmost
    been_following = (c.now() - c.last_ref_t) < c.p.role_memory
    return (not been_following) or frontmost(c)


# ----------------------------- behaviours -----------------------------
class FrontBlocked(py_trees.behaviour.Behaviour):
    def __init__(self, ctx): super().__init__('FrontBlocked'); self.c = ctx
    def update(self):
        return S.SUCCESS if self.c.front < self.c.p.front_stop else S.FAILURE


class Avoid(py_trees.behaviour.Behaviour):
    def __init__(self, ctx): super().__init__('Avoid'); self.c = ctx
    def update(self):
        tw = Twist(); tw.angular.z = self.c.p.turn_speed
        self.c.cmd.publish(tw); return S.RUNNING


class HaveReference(py_trees.behaviour.Behaviour):
    def __init__(self, ctx): super().__init__('HaveReference'); self.c = ctx
    def update(self):
        return S.SUCCESS if have_reference(self.c) else S.FAILURE


class InPatrolMode(py_trees.behaviour.Behaviour):
    """Gate: the follow/recover/lead subtree only runs in a patrol formation, not random."""
    def __init__(self, ctx): super().__init__('InPatrolMode'); self.c = ctx
    def update(self):
        return S.SUCCESS if self.c.formation in ('convoy', 'parallel') else S.FAILURE


class ShouldFollow(py_trees.behaviour.Behaviour):
    """I should be in formation (not the leader) but currently have no reference -> recover.
    Disabled when enable_recovery is false (then a lost unit falls through to Lead, the
    pre-recovery behaviour) — handy for the tutorial's 'patrol without recovery' phase."""
    def __init__(self, ctx): super().__init__('ShouldFollow'); self.c = ctx
    def update(self):
        if not self.c.p.enable_recovery:
            return S.FAILURE
        return S.FAILURE if is_leader_role(self.c) else S.SUCCESS


class RandomWalk(py_trees.behaviour.Behaviour):
    """Wander: resample a random (v, w) every walk_period. Obstacle avoidance is the higher
    'safety' branch (FrontBlocked -> Avoid), so this stays a pure random cruise."""
    def __init__(self, ctx): super().__init__('RandomWalk'); self.c = ctx
    def update(self):
        c, p = self.c, self.c.p
        if c.now() >= c.t_next_sample:
            c.walk_v = random.uniform(0.08, 0.20)
            c.walk_w = random.uniform(-0.4, 0.4)
            c.t_next_sample = c.now() + p.walk_period
        tw = Twist(); tw.linear.x = c.walk_v; tw.angular.z = c.walk_w
        c.cmd.publish(tw); return S.RUNNING


class HoldFormation(py_trees.behaviour.Behaviour):
    def __init__(self, ctx): super().__init__('HoldFormation'); self.c = ctx
    def update(self):
        c, p = self.c, self.c.p
        c.search_t0 = 0.0                                # reacquired -> reset search timer
        tw = Twist()
        if c.formation == 'convoy':
            fresh = (c.now() - c.back_t) < p.marker_timeout
            if fresh and c.front < 2.5:
                d = p.us_alpha * c.front + (1 - p.us_alpha) * c.back_range
            elif fresh:
                d = c.back_range
            else:
                d = c.front
            tw.linear.x = max(-0.1, min(p.cruise, p.kv * (d - p.follow_distance)))
            tw.angular.z = -p.kw_bear * c.back_bearing
        else:
            tw.linear.x = c.anchor.linear.x
            gap = c.left if p.gap_side == 'left' else c.right
            gap_err = gap - p.lateral_gap if gap < 2.5 else 0.0
            sign = 1.0 if p.gap_side == 'left' else -1.0
            tw.angular.z = c.anchor.angular.z + sign * p.k_lat * gap_err
        c.cmd.publish(tw); return S.RUNNING


class SearchAndRecover(py_trees.behaviour.Behaviour):
    """Vision-only: spin 360 to reacquire any peer/anchor marker; expand; time out -> lead."""
    def __init__(self, ctx): super().__init__('SearchAndRecover'); self.c = ctx
    def update(self):
        c, p = self.c, self.c.p
        if c.search_t0 == 0.0:
            c.search_t0 = c.now()
        elapsed = c.now() - c.search_t0
        if elapsed > p.search_timeout:                  # give up -> let Lead take over
            c.last_ref_t = -1e9                          # no longer "in formation"
            return S.FAILURE
        tw = Twist()
        spin_period = 2 * math.pi / p.search_turn
        phase = elapsed % (spin_period + p.creep_time)
        if phase < spin_period:
            tw.angular.z = p.search_turn                 # a full 360 scan
        else:
            tw.linear.x = p.cruise * 0.5                 # creep to widen the search
        c.cmd.publish(tw); return S.RUNNING


class Lead(py_trees.behaviour.Behaviour):
    def __init__(self, ctx): super().__init__('Lead'); self.c = ctx; self.turn_until = 0.0
    def update(self):
        c, p = self.c, self.c.p
        tw = Twist()
        if c.now() < self.turn_until:
            tw.angular.z = p.turn_speed
        elif c.front < p.sweep_clear:
            self.turn_until = c.now() + p.turn_time
            tw.angular.z = p.turn_speed
        else:
            tw.linear.x = p.cruise
        c.cmd.publish(tw); return S.RUNNING


# ----------------------------- node -----------------------------
class PatrolBT(Node):
    def __init__(self):
        super().__init__('patrol_bt')
        self.declare_parameter('formation', 'convoy')   # random | convoy | parallel
        self.declare_parameter('leader_ns', 'r1')
        self.declare_parameter('gap_side', 'right')
        self.declare_parameter('enable_recovery', True)
        self.declare_parameter('debug_tree', False)      # log the live ASCII tree each second
        me = self.get_namespace().strip('/')

        p = SimpleNamespace(
            front_stop=0.25, turn_speed=0.8, cruise=0.16, walk_period=2.0,
            follow_distance=0.5, kv=0.6, kw_bear=1.2, us_alpha=0.7, marker_timeout=0.8,
            lateral_gap=0.6, k_lat=1.0, sweep_clear=0.6, turn_time=2.0,
            role_memory=8.0, anchor_fresh=5.0, search_timeout=25.0, creep_time=1.0,
            search_turn=0.7,
            enable_recovery=self.get_parameter('enable_recovery').value,
            leader_ns=self.get_parameter('leader_ns').value,
            gap_side=self.get_parameter('gap_side').value)

        self.ctx = SimpleNamespace(
            p=p, me=me, formation=self.get_parameter('formation').value,
            front=9.9, left=9.9, right=9.9,
            back_range=9.9, back_bearing=0.0, back_t=-9.9,
            right_range=9.9, right_bearing=0.0, right_t=-9.9,
            last_ref_t=-1e9, search_t0=0.0, anchor=Twist(), anchor_range={},
            walk_v=0.0, walk_w=0.0, t_next_sample=0.0,
            cmd=self.create_publisher(Twist, 'cmd_vel', 10),
            now=lambda: self.get_clock().now().nanoseconds * 1e-9)

        self.create_subscription(Range, 'ultrasonic/front',
                                 lambda m: setattr(self.ctx, 'front', m.range), 10)
        self.create_subscription(Range, 'ultrasonic/left',
                                 lambda m: setattr(self.ctx, 'left', m.range), 10)
        self.create_subscription(Range, 'ultrasonic/right',
                                 lambda m: setattr(self.ctx, 'right', m.range), 10)
        self.create_subscription(Detection2DArray, 'aruco/detections', self.on_dets, 10)
        self.create_subscription(TwistStamped, f'/{p.leader_ns}/formation/anchor',
                                 lambda m: setattr(self.ctx, 'anchor', m.twist), 10)
        # every unit's anchor distance (for the leader/lost decision)
        for u in UNITS:
            self.create_subscription(Float32, f'/{u}/anchor_range',
                                     lambda m, uu=u: self.ctx.anchor_range.__setitem__(
                                         uu, (m.data, self.ctx.now())), 10)
        self.create_service(SetFormation, 'set_formation', self.on_set_formation)

        # patrol subtree: follow -> recover -> lead (only ticked in a patrol formation)
        patrol = py_trees.composites.Selector('patrol_modes', memory=False, children=[
            py_trees.composites.Sequence('follow', memory=False,
                                         children=[HaveReference(self.ctx), HoldFormation(self.ctx)]),
            py_trees.composites.Sequence('recover', memory=False,
                                         children=[ShouldFollow(self.ctx), SearchAndRecover(self.ctx)]),
            Lead(self.ctx)])
        root = py_trees.composites.Selector('patrol_root', memory=False, children=[
            py_trees.composites.Sequence('safety', memory=False,
                                         children=[FrontBlocked(self.ctx), Avoid(self.ctx)]),
            py_trees.composites.Sequence('patrol', memory=False,
                                         children=[InPatrolMode(self.ctx), patrol]),
            RandomWalk(self.ctx)])     # fallback: random formation (or no patrol)

        # wrap in a py_trees_ros BehaviourTree so `py-trees-tree-watcher` can show it live
        self.tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        self.tree.setup(node=self, timeout=15.0)
        self.debug_tree = self.get_parameter('debug_tree').value
        self.create_timer(0.1, self.tick)
        if self.debug_tree:
            self.create_timer(1.0, self.log_tree)
        self.get_logger().info(f'patrol_bt up (ns={me}, formation={self.ctx.formation}, '
                               f'leader={p.leader_ns}, recovery={p.enable_recovery})')

    def on_dets(self, msg):
        my_unit = self.ctx.me
        my_id = int(my_unit[1:]) if my_unit[1:].isdigit() else -1
        for d in msg.detections:
            if not d.results:
                continue
            mid = int(d.results[0].hypothesis.class_id)
            unit, face = mid // 10, mid % 10
            if unit == my_id or mid == 99:
                continue
            pos = d.results[0].pose.pose.position
            rng = float(math.hypot(pos.x, pos.z)) or pos.z
            bearing = math.atan2(pos.x, pos.z)
            if face == 0:
                self.ctx.back_range, self.ctx.back_bearing = rng, bearing
                self.ctx.back_t = self.ctx.now()
            elif face == 1:
                self.ctx.right_range, self.ctx.right_bearing = rng, bearing
                self.ctx.right_t = self.ctx.now()
            self.ctx.last_ref_t = self.ctx.now()        # role memory: I'm in formation

    def on_set_formation(self, req, resp):
        if req.formation not in ('random', 'convoy', 'parallel'):
            resp.success = False
            resp.message = "formation must be 'random', 'convoy' or 'parallel'"
            return resp
        self.ctx.formation = req.formation
        resp.success = True; resp.message = f'formation -> {req.formation}'
        self.get_logger().info(resp.message); return resp

    def tick(self):
        self.tree.tick()

    def log_tree(self):
        self.get_logger().info('\n' + py_trees.display.unicode_tree(
            self.tree.root, show_status=True))


def main(args=None):
    rclpy.init(args=args)
    node = PatrolBT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
