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


def _solve_tridiagonal(a: List[float], b: List[float], c: List[float], d: List[float]) -> List[float]:
	"""Solve tridiagonal system Ax = d with vectors a (sub), b (diag), c (super).

	Uses the Thomas algorithm. All lists are 0-indexed and length n, where
	a[0] and c[-1] are unused (can be 0). Returns solution x of length n.
	"""
	n = len(d)
	if n == 0:
		return []
	# Copy arrays to avoid modifying inputs
	cp = c.copy()
	dp = d.copy()
	bp = b.copy()

	# Forward sweep
	for i in range(1, n):
		if bp[i - 1] == 0.0:
			raise ZeroDivisionError("Zero diagonal on tridiagonal solve")
		m = a[i] / bp[i - 1]
		bp[i] = bp[i] - m * cp[i - 1]
		dp[i] = dp[i] - m * dp[i - 1]

	# Back substitution
	x = [0.0] * n
	if bp[-1] == 0.0:
		raise ZeroDivisionError("Zero diagonal on tridiagonal solve")
	x[-1] = dp[-1] / bp[-1]
	for i in range(n - 2, -1, -1):
		x[i] = (dp[i] - cp[i] * x[i + 1]) / bp[i]
	return x


def _natural_cubic_second_derivatives(t: List[float], y: List[float]) -> List[float]:
	"""Compute second derivatives (M) for natural cubic spline on (t,y).

	Returns list M of length n with M[0]=M[-1]=0.
	"""
	n = len(t)
	if n <= 2:
		return [0.0] * n

	h = [t[i + 1] - t[i] for i in range(n - 1)]
	# build tridiagonal system for interior points 1..n-2
	a = [0.0] * (n - 2)
	b = [0.0] * (n - 2)
	c = [0.0] * (n - 2)
	d = [0.0] * (n - 2)
	for i in range(1, n - 1):
		idx = i - 1
		a[idx] = h[i - 1]
		b[idx] = 2.0 * (h[i - 1] + h[i])
		c[idx] = h[i]
		d[idx] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])

	# Solve for interior second derivatives
	M_interior = _solve_tridiagonal(a, b, c, d)

	M = [0.0] * n
	for i in range(1, n - 1):
		M[i] = M_interior[i - 1]
	return M


def _evaluate_spline_on_segment(ti: float, ti1: float, yi: float, yi1: float, Mi: float, Mi1: float, t: float) -> float:
	"""Evaluate natural cubic spline segment between ti..ti1 at parameter t."""
	h = ti1 - ti
	if h == 0:
		return yi
	A = (ti1 - t) / h
	B = (t - ti) / h
	term1 = (Mi * (A ** 3) * (h * h) ) / 6.0
	term2 = (Mi1 * (B ** 3) * (h * h) ) / 6.0
	term3 = (yi - (Mi * h * h) / 6.0) * A
	term4 = (yi1 - (Mi1 * h * h) / 6.0) * B
	return term1 + term2 + term3 + term4


def cubic_spline(path: List[Tuple[float, float]], samples_per_segment: int = 20) -> List[Tuple[float, float]]:
	"""Return sampled points along a natural cubic spline through `path` points.

	The spline is parameterized by chord length (cumulative distance), and
	x(t) and y(t) are interpolated separately using natural cubic splines.
	"""
	if not path:
		return []
	n = len(path)
	if n == 1:
		return [(float(path[0][0]), float(path[0][1]))]

	# parameterize by chord length
	t: List[float] = [0.0]
	for i in range(1, n):
		dx = path[i][0] - path[i - 1][0]
		dy = path[i][1] - path[i - 1][1]
		t.append(t[-1] + math.hypot(dx, dy))

	# handle degenerate case all points coincide
	if t[-1] == 0.0:
		return [(float(path[0][0]), float(path[0][1])) for _ in range(samples_per_segment + 1)]

	xs = [float(p[0]) for p in path]
	ys = [float(p[1]) for p in path]

	Mx = _natural_cubic_second_derivatives(t, xs)
	My = _natural_cubic_second_derivatives(t, ys)

	out: List[Tuple[float, float]] = []
	for i in range(n - 1):
		ti = t[i]
		ti1 = t[i + 1]
		for k in range(samples_per_segment):
			u = k / float(samples_per_segment)
			tt = ti + u * (ti1 - ti)
			x = _evaluate_spline_on_segment(ti, ti1, xs[i], xs[i + 1], Mx[i], Mx[i + 1], tt)
			y = _evaluate_spline_on_segment(ti, ti1, ys[i], ys[i + 1], My[i], My[i + 1], tt)
			out.append((x, y))
	# append final point exactly
	out.append((xs[-1], ys[-1]))
	return out


def plot_path_with_cubic(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], cubic_pts: Optional[List[Tuple[float, float]]], fname: str):
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

	# Plot cubic spline
	if cubic_pts:
		cx = [p[0] for p in cubic_pts]
		cy = [p[1] for p in cubic_pts]
		ax.plot(cx, cy, '-', color='green', linewidth=2, label='cubic spline')

	ax.set_xlim(-0.5, width + 0.5)
	ax.set_ylim(-0.5, height + 0.5)
	ax.set_aspect('equal')
	ax.set_title('Lazy Theta* Path (with Cubic spline)')
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
		
	# Create and save cubic-spline-smoothed version
	cubic_pts = cubic_spline(path, samples_per_segment=30)
	cubic_fname = 'lazy_theta_star_path_cubic.png'
	plot_path_with_cubic(length, width, obstacles, path, cubic_pts, cubic_fname)
	print(f'Cubic-spline path image saved to {cubic_fname}')

