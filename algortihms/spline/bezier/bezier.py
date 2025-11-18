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


def _cubic_bezier_point(p0: Tuple[float, float], c1: Tuple[float, float], c2: Tuple[float, float], p3: Tuple[float, float], t: float) -> Tuple[float, float]:
	"""Evaluate a cubic Bezier at parameter t in [0,1]."""
	u = 1.0 - t
	uu = u * u
	uuu = uu * u
	tt = t * t
	ttt = tt * t
	x = uuu * p0[0] + 3 * uu * t * c1[0] + 3 * u * tt * c2[0] + ttt * p3[0]
	y = uuu * p0[1] + 3 * uu * t * c1[1] + 3 * u * tt * c2[1] + ttt * p3[1]
	return (x, y)


def _catmull_rom_to_bezier_segments(points: List[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]]:
	"""Convert a polyline (points) to a list of cubic Bezier segments.

	Uses the common Catmull-Rom to Bezier conversion so the resulting curve
	passes through the original points and is C1 continuous.
	"""
	n = len(points)
	if n < 2:
		return []
	segments: List[Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]] = []
	for i in range(n - 1):
		p0 = points[i - 1] if i - 1 >= 0 else points[i]
		p1 = points[i]
		p2 = points[i + 1]
		p3 = points[i + 2] if i + 2 < n else points[i + 1]

		# Tangent-based control points (tension = 1/6 gives Catmull-Rom style)
		c1 = (p1[0] + (p2[0] - p0[0]) / 6.0, p1[1] + (p2[1] - p0[1]) / 6.0)
		c2 = (p2[0] - (p3[0] - p1[0]) / 6.0, p2[1] - (p3[1] - p1[1]) / 6.0)
		segments.append((p1, c1, c2, p2))

	return segments


def bezier_spline(path: List[Tuple[float, float]], samples_per_segment: int = 20) -> List[Tuple[float, float]]:
	"""Convert a polyline `path` into a smooth composite cubic Bezier curve.

	Returns a list of sampled points along the composite Bezier curve. The
	algorithm converts the polyline into cubic segments using a Catmull-Rom
	style conversion and samples each segment uniformly.
	"""
	if not path:
		return []
	if len(path) < 2:
		return [(float(path[0][0]), float(path[0][1]))]

	segments = _catmull_rom_to_bezier_segments(path)
	out: List[Tuple[float, float]] = []
	for (p0, c1, c2, p3) in segments:
		for i in range(samples_per_segment):
			t = i / float(samples_per_segment)
			out.append(_cubic_bezier_point(p0, c1, c2, p3, t))
	# ensure last point is included exactly
	out.append((float(path[-1][0]), float(path[-1][1])))
	return out


def plot_path_with_bezier(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], bezier_pts: Optional[List[Tuple[float, float]]], fname: str):
	"""Plot the raw grid, obstacles, the original path and optional Bezier smoothing."""
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

	# Plot bezier
	if bezier_pts:
		bx = [p[0] for p in bezier_pts]
		by = [p[1] for p in bezier_pts]
		ax.plot(bx, by, '-', color='orange', linewidth=2, label='bezier')

	ax.set_xlim(-0.5, width + 0.5)
	ax.set_ylim(-0.5, height + 0.5)
	ax.set_aspect('equal')
	ax.set_title('Lazy Theta* Path (with Bezier smoothing)')
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

	# Create and save Bezier-smoothed version
	bezier_pts = bezier_spline(path, samples_per_segment=30)
	bezier_fname = 'lazy_theta_star_path_bezier.png'
	plot_path_with_bezier(length, width, obstacles, bezier_pts, bezier_fname)
	print(f'Bezier-smoothed path image saved to {bezier_fname}')

