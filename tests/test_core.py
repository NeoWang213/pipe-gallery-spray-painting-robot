import math
import unittest
from software.planning import rectangle_path, circle_path, project_path, map_2d
from software.execution import TrajectoryExecutor, Outcome
from software.settings import CONFIG


class PlanningTests(unittest.TestCase):
    def test_rectangle_covers_edges_and_alternates(self):
        points = rectangle_path(85, 93, 10, 12, 20)
        self.assertEqual(points[0], (10, 12))
        self.assertEqual(points[-1][1], 93)
        for i in range(0, len(points), 2):
            a, b = points[i:i+2]
            self.assertEqual(a[1], b[1])
            self.assertEqual({a[0], b[0]}, {10, 85})
            self.assertEqual(a[0], 10 if i//2 % 2 == 0 else 85)
            if i:
                self.assertLessEqual(a[1]-points[i-2][1], 20+1e-9)
                self.assertEqual(a[0], points[i-1][0])

    def test_tiny_rectangle(self):
        self.assertEqual(len(rectangle_path(1, 1, 2, 2, 20)), 4)

    def test_invalid_rectangle(self):
        for values in [(0,0,0,2,1),(0,0,2,2,0),(0,0,2,2,-1),(0,0,math.nan,2,1)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                rectangle_path(*values)

    def test_circle_inside_selected_disk(self):
        points = circle_path(100, 100, 33, 10)
        self.assertTrue(all(math.hypot(x-100,y-100) <= 33+1e-9 for x,y in points))
        self.assertAlmostEqual(max(math.hypot(x-100,y-100) for x,y in points), 33)

    def test_invalid_circle(self):
        for r, step in [(0,2),(-1,2),(5,0),(math.inf,2)]:
            with self.subTest(r=r,step=step), self.assertRaises(ValueError):
                circle_path(0,0,r,step)

    def test_depth_projection_matches_experimental_mapping(self):
        points, depth = project_path([(320,240),(420,340)],lambda x,y:500,
            {'fx':500,'fy':500,'cx':320,'cy':240},CONFIG['mapping_3d'])
        self.assertEqual(points,[(300,50,600),(350,0,600)])
        self.assertEqual(depth,500)

    def test_bad_depth_rejects_whole_path(self):
        for depth in [None,0,-1,math.nan,math.inf,2001]:
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                project_path([(10,10),(20,20)],lambda x,y:500 if x==10 else depth,
                    {'fx':500,'fy':500,'cx':0,'cy':0},CONFIG['mapping_3d'])

    def test_no_silent_2d_clipping(self):
        self.assertEqual(map_2d(320,240,CONFIG['mapping_2d']),(300,0))
        with self.assertRaises(ValueError):
            map_2d(-1000,0,CONFIG['mapping_2d'])


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.runner=TrajectoryExecutor()
        self.points=[(1,2,3),(4,5,6),(7,8,9)]
        self.calls=[]

    def move(self,*p):
        self.calls.append(p)
        return True

    def test_complete(self):
        self.assertEqual(self.runner.execute(self.points,self.move,lambda:True),Outcome.COMPLETE)
        self.assertEqual(self.calls,self.points)

    def test_disconnected_issues_no_motion(self):
        self.assertEqual(self.runner.execute(self.points,self.move,lambda:False),Outcome.DISCONNECTED)
        self.assertEqual(self.calls,[])

    def test_failure_stops_at_first_point(self):
        def fail(*p):
            self.calls.append(p)
            return False
        self.assertEqual(self.runner.execute(self.points,fail,lambda:True),Outcome.FAILED)
        self.assertEqual(self.calls,[self.points[0]])

    def test_cancel_in_motion_is_not_success(self):
        def cancel(*p):
            self.calls.append(p)
            self.runner.cancel()
            return True
        self.assertEqual(self.runner.execute(self.points,cancel,lambda:True),Outcome.STOPPED)
        self.assertEqual(self.calls,[self.points[0]])

    def test_pre_cancel_does_not_move(self):
        self.runner.cancel()
        self.assertEqual(self.runner.execute(self.points,self.move,lambda:True),Outcome.STOPPED)
        self.assertEqual(self.calls,[])

    def test_disconnect_between_points(self):
        connected=[True]
        def drop(*p):
            connected[0]=False
            return self.move(*p)
        self.assertEqual(self.runner.execute(self.points,drop,lambda:connected[0]),Outcome.DISCONNECTED)
        self.assertEqual(len(self.calls),1)

    def test_invalid_path_never_moves(self):
        for points in [[],[(1,2,3),(4,math.nan,6)],[(1,2)]]:
            with self.subTest(points=points):
                self.assertEqual(self.runner.execute(points,self.move,lambda:True),Outcome.INVALID)
        self.assertEqual(self.calls,[])

    def test_exception_is_failure(self):
        def fail(*p):
            raise OSError('mock controller error')
        self.assertEqual(self.runner.execute(self.points,fail,lambda:True),Outcome.FAILED)


if __name__=='__main__':
    unittest.main()
