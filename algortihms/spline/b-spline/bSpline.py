import math
import heapq
import matplotlib.pyplot as plt
from typing import Tuple, Dict, Optional, List, Set


class Node:
	def __init__(self, x: int, y: int):
		self.x = x
		self.y = y
		self.g = float('inf')
		self.h = 0.0
		self.parent: Optional['Node'] = None

	def f(self):
		return self.g + self.h

	def coord(self) -> Tuple[int, int]:
		return (self.x, self.y)


def euclidean(a: Tuple[int, int], b: Tuple[int, int]) -> float:
	return math.hypot(a[0] - b[0], a[1] - b[1])


def bresenham_line(a: Tuple[int, int], b: Tuple[int, int]) -> List[Tuple[int, int]]:
	"""Return integer grid cells intersected by the line from a to b (inclusive)."""
	(x0, y0) = a
	(x1, y1) = b
	points: List[Tuple[int, int]] = []
	dx = abs(x1 - x0)
	dy = abs(y1 - y0)
	x, y = x0, y0
	sx = 1 if x0 < x1 else -1
	sy = 1 if y0 < y1 else -1
	if dx >= dy:
		err = dx / 2.0
		while x != x1:
			points.append((x, y))
			err -= dy
			if err < 0:
				y += sy
				err += dx
			x += sx
		points.append((x1, y1))
	else:
		err = dy / 2.0
		while y != y1:
			points.append((x, y))
			err -= dx
			if err < 0:
				x += sx
				err += dy
			y += sy
		points.append((x1, y1))
	return points


class LazyThetaStar:
	def __init__(self, width: int, height: int, obstacles: Set[Tuple[int, int]]):
		self.width = width
		self.height = height
		self.obstacles = set(obstacles)
		self.nodes: Dict[Tuple[int, int], Node] = {}
		for x in range(0, width + 1):
			for y in range(0, height + 1):
				self.nodes[(x, y)] = Node(x, y)

	def in_bounds(self, coord: Tuple[int, int]) -> bool:
		x, y = coord
		return 0 <= x <= self.width and 0 <= y <= self.height

	def is_free(self, coord: Tuple[int, int]) -> bool:
		return coord not in self.obstacles

	def neighbors(self, coord: Tuple[int, int]) -> List[Tuple[int, int]]:
		x, y = coord
		nbrs: List[Tuple[int, int]] = []
		for dx in (-1, 0, 1):
			for dy in (-1, 0, 1):
				if dx == 0 and dy == 0:
					continue
				nx, ny = x + dx, y + dy
				if 0 <= nx <= self.width and 0 <= ny <= self.height and self.is_free((nx, ny)):
					nbrs.append((nx, ny))
		return nbrs

	def line_of_sight(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
		# Use Bresenham to check discrete cells intersected by the line
		for p in bresenham_line(a, b):
			if p in self.obstacles:
				# allow endpoints if they are start or goal (shouldn't be obstacles)
				if p != a and p != b:
					return False
		return True

	def compute_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[float, float]]:
		if start in self.obstacles or goal in self.obstacles:
			raise ValueError("Start or goal is inside an obstacle")

		# Initialize
		for node in self.nodes.values():
			node.g = float('inf')
			node.parent = None

		start_node = self.nodes[start]
		goal_node = self.nodes[goal]
		start_node.g = 0.0
		start_node.parent = start_node
		start_node.h = euclidean(start, goal)

		open_heap: List[Tuple[float, Tuple[int, int]]] = []
		entry_f = start_node.f()
		heapq.heappush(open_heap, (entry_f, start))
		closed: Set[Tuple[int, int]] = set()

		while open_heap:
			_, s_coord = heapq.heappop(open_heap)
			s_node = self.nodes[s_coord]
			if s_coord in closed:
				continue
			if s_coord == goal:
				break
			closed.add(s_coord)

			for sn_coord in self.neighbors(s_coord):
				sn = self.nodes[sn_coord]
				if sn_coord not in self.nodes:
					continue

				# If line of sight between parent(s) and s' then consider shortcut
				s_parent = s_node.parent.coord() if isinstance(s_node.parent, Node) else s_node.parent
				if s_node.parent is not None and self.line_of_sight(s_node.parent.coord(), sn_coord):
					# Path length via s.parent
					tentative_g = s_node.parent.g + euclidean(s_node.parent.coord(), sn_coord)
					if tentative_g < sn.g:
						sn.g = tentative_g
						sn.parent = s_node.parent
						sn.h = euclidean(sn_coord, goal)
						heapq.heappush(open_heap, (sn.f(), sn_coord))
				else:
					# No line of sight: standard relaxation via s
					tentative_g = s_node.g + euclidean(s_coord, sn_coord)
					if tentative_g < sn.g:
						sn.g = tentative_g
						sn.parent = s_node
						sn.h = euclidean(sn_coord, goal)
						heapq.heappush(open_heap, (sn.f(), sn_coord))

		# Reconstruct path
		path: List[Tuple[float, float]] = []
		cur = goal
		if self.nodes[goal].parent is None:
			return []
		while cur != start:
			node = self.nodes[cur]
			path.append((node.x, node.y))
			if node.parent is None:
				break
			# parent may be Node or reference to Node
			if isinstance(node.parent, Node):
				cur = node.parent.coord()
			else:
				cur = node.parent
		path.append(start)
		path.reverse()
		return path


def plot_path(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], fname: str):
	fig, ax = plt.subplots(figsize=(8, 4))
	# Plot grid
	for x in range(width + 1):
		ax.plot([x, x], [0, height], color='lightgray', linewidth=0.5)
	for y in range(height + 1):
		ax.plot([0, width], [y, y], color='lightgray', linewidth=0.5)

	# Plot obstacles as red squares
	ox = [o[0] for o in obstacles]
	oy = [o[1] for o in obstacles]
	ax.scatter(ox, oy, marker='s', s=200, color='red', label='obstacle')

	# Plot path
	if path:
		px = [p[0] for p in path]
		py = [p[1] for p in path]
		ax.plot(px, py, '-o', color='blue', linewidth=2, markersize=6, label='path')

	ax.set_xlim(-0.5, width + 0.5)
	ax.set_ylim(-0.5, height + 0.5)
	ax.set_aspect('equal')
	ax.set_title('Lazy Theta* Path')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.legend()
	plt.grid(False)
	plt.savefig(fname, dpi=200, bbox_inches='tight')
	plt.close(fig)


def _make_knot_vector(n_ctrl: int, degree: int, clamped: bool = True) -> List[float]:
	"""Create a uniform knot vector for n_ctrl control points and given degree.

	Returns a normalized knot vector in [0,1]. If `clamped` is True, an open
	uniform (clamped) knot vector is returned so curve interpolates endpoints.
	"""
	if n_ctrl <= degree:
		# minimal degenerate knot vector
		return [0.0] * (n_ctrl + degree + 1)

	m = n_ctrl + degree + 1
	if clamped:
		# open uniform: first degree+1 zeros, last degree+1 ones, interior uniform
		interior_count = m - 2 * (degree + 1)
		knots = [0.0] * (degree + 1)
		if interior_count > 0:
			for i in range(1, interior_count + 1):
				knots.append(i / float(interior_count + 1))
		knots += [1.0] * (degree + 1)
	else:
		# non-clamped uniform from 0..1
		knots = [i / float(m - 1) for i in range(m)]

	return knots


def _cox_de_boor(u: float, i: int, k: int, knots: List[float]) -> float:
	"""Cox-De Boor recursion for B-spline basis function N_{i,k}(u).

	u: parameter
	i: basis index
	k: degree
	knots: knot vector
	"""
	# base case degree 0
	if k == 0:
		# special handling for u == last knot
		if knots[i] <= u < knots[i + 1]:
			return 1.0
		# include last endpoint
		if u == knots[-1] and knots[i + 1] == knots[-1] and knots[i] < knots[i + 1]:
			return 1.0
		return 0.0

	denom1 = knots[i + k] - knots[i]
	term1 = 0.0
	if denom1 != 0.0:
		term1 = ((u - knots[i]) / denom1) * _cox_de_boor(u, i, k - 1, knots)

	denom2 = knots[i + k + 1] - knots[i + 1]
	term2 = 0.0
	if denom2 != 0.0:
		term2 = ((knots[i + k + 1] - u) / denom2) * _cox_de_boor(u, i + 1, k - 1, knots)

	return term1 + term2


def bspline_curve(ctrl_pts: List[Tuple[float, float]], degree: int = 3, samples: int = 200, clamped: bool = True) -> List[Tuple[float, float]]:
	"""Sample a B-spline curve defined by control points `ctrl_pts`.

	- `degree` is the spline degree (cubic by default).
	- `samples` is number of sample points along parameter domain [0,1].
	- `clamped` selects open-uniform knot vector to interpolate endpoints.
	Returns list of sampled (x,y) points.
	"""
	n = len(ctrl_pts)
	if n == 0:
		return []
	if n == 1:
		return [(float(ctrl_pts[0][0]), float(ctrl_pts[0][1]))]

	# if too few control points for degree, lower degree
	deg = min(degree, n - 1)
	knots = _make_knot_vector(n, deg, clamped=clamped)

	out: List[Tuple[float, float]] = []
	if samples <= 0:
		samples = 200

	u_start = knots[deg]
	u_end = knots[-deg - 1]
	if u_end < u_start:
		u_start, u_end = knots[0], knots[-1]

	for si in range(samples + 1):
		u = u_start + (u_end - u_start) * (si / float(samples))
		# Ensure u within domain
		if u < knots[0]:
			u = knots[0]
		if u > knots[-1]:
			u = knots[-1]

		x = 0.0
		y = 0.0
		for i in range(n):
			Ni = _cox_de_boor(u, i, deg, knots)
			x += Ni * float(ctrl_pts[i][0])
			y += Ni * float(ctrl_pts[i][1])
		out.append((x, y))

	return out


def plot_path_with_bspline(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], bspline_pts: Optional[List[Tuple[float, float]]], fname: str):
	fig, ax = plt.subplots(figsize=(8, 4))
	# Plot grid
	for x in range(width + 1):
		ax.plot([x, x], [0, height], color='lightgray', linewidth=0.5)
	for y in range(height + 1):
		ax.plot([0, width], [y, y], color='lightgray', linewidth=0.5)

	# Plot obstacles as red squares
	ox = [o[0] for o in obstacles]
	oy = [o[1] for o in obstacles]
	ax.scatter(ox, oy, marker='s', s=200, color='red', label='obstacle')

	# Plot path
	if path:
		px = [p[0] for p in path]
		py = [p[1] for p in path]
		ax.plot(px, py, '-o', color='blue', linewidth=2, markersize=6, label='raw path')

	# Plot B-spline
	if bspline_pts:
		bx = [p[0] for p in bspline_pts]
		by = [p[1] for p in bspline_pts]
		ax.plot(bx, by, '-', color='purple', linewidth=2, label='B-spline')

	ax.set_xlim(-0.5, width + 0.5)
	ax.set_ylim(-0.5, height + 0.5)
	ax.set_aspect('equal')
	ax.set_title('Lazy Theta* Path (with B-spline)')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.legend()
	plt.grid(False)
	plt.savefig(fname, dpi=200, bbox_inches='tight')
	plt.close(fig)


if __name__ == '__main__':
	# Define area: length 8 (x), width 4 (y)
	length = 8
	width = 4
	# Obstacle coordinates
	obstacles = {(2, 1), (5, 2)}

	lts = LazyThetaStar(length, width, obstacles)
	start = (0, 0)
	goal = (8, 4)
	path = lts.compute_path(start, goal)
	if not path:
		print('No path found')
	else:
		print('Path found with {} nodes:'.format(len(path)))
		print(path)

	# Create and save B-spline-smoothed version (cubic by default)
	bspline_pts = bspline_curve(path, degree=3, samples=200, clamped=True)
	bspline_fname = 'lazy_theta_star_path_bspline.png'
	plot_path_with_bspline(length, width, obstacles, path, bspline_pts, bspline_fname)
	print(f'B-spline path image saved to {bspline_fname}')

