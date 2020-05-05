"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
import os
import sys
import time
from enum import IntEnum
from grid_map import GridMap as gm

import matplotlib.pyplot as plt
import numpy as np
from grid_map_lib import GridMap

do_animation = True

def plot_robot(pose, params):
	r = params.sensor_range_m
	plt.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
			 [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
	plt.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
			 [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')
	plt.plot(pose[0], pose[1], 'ro', markersize=5)
	plt.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
			  head_length=0.1, head_width=0.1)

def visualize(traj, pose, params):
	plt.plot(traj[:,0], traj[:,1], 'g')
	plot_robot(pose, params)
	plt.legend()

def motion(state, goal, params):
	# state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
	dx = goal[0] - state[0]
	dy = goal[1] - state[1]
	goal_yaw = math.atan2(dy, dx)
	K_theta = 3
	state[4] = K_theta*math.sin(goal_yaw - state[2]) # omega(rad/s)
	state[2] += params.dt*state[4] # yaw(rad)

	dist_to_goal = np.linalg.norm(goal - state[:2])
	K_v = 0.1
	state[3] += K_v*dist_to_goal
	if state[3] >= params.max_vel: state[3] = params.max_vel
	if state[3] <= params.min_vel: state[3] = params.min_vel

	dv = params.dt*state[3]
	state[0] += dv*np.cos(state[2]) # x(m)
	state[1] += dv*np.sin(state[2]) # y(m)

	return state

class Params:
	def __init__(self):
		self.numiters = 1000
		self.animate = 1
		self.dt = 0.1
		self.goal_tol = 0.15
		self.max_vel = 0.5 # m/s
		self.min_vel = 0.1 # m/s
		self.sensor_range_m = 0.3 # m
		self.time_to_switch_goal = 5.0 # sec
		#self.sweep_resolution = 15 #0.25 # m

class SweepSearcher:
	class SweepDirection(IntEnum):
		UP = 1
		DOWN = -1

	class MovingDirection(IntEnum):
		RIGHT = 1
		LEFT = -1

	def __init__(self, mdirection, sdirection, xinds_goaly, goaly):
		self.moving_direction = mdirection
		self.sweep_direction = sdirection
		self.turing_window = []
		self.update_turning_window()
		self.xinds_goaly = xinds_goaly
		self.goaly = goaly

	def move_target_grid(self, cxind, cyind, gmap):
		nxind = self.moving_direction + cxind
		nyind = cyind

		# found safe grid
		if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5): #0.5 changed
			return nxind, nyind
		else:  # occupided
			ncxind, ncyind = self.find_safe_turning_grid(cxind, cyind, gmap)
			if (ncxind is None) and (ncyind is None):
				# moving backward
				ncxind = -self.moving_direction + cxind
				ncyind = cyind
				if gmap.check_occupied_from_xy_index(ncxind, ncyind):
					# moved backward, but the grid is occupied by obstacle
					return None, None
			else:
				# keep moving until end
				while not gmap.check_occupied_from_xy_index(ncxind + self.moving_direction, ncyind, occupied_val=0.5):
					ncxind += self.moving_direction
				self.swap_moving_direction()
			return ncxind, ncyind

	def find_safe_turning_grid(self, cxind, cyind, gmap):

		for (dxind, dyind) in self.turing_window:

			nxind = dxind + cxind
			nyind = dyind + cyind

			# found safe grid
			if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):
				return nxind, nyind

		return None, None

	def is_search_done(self, gmap):
		for ix in self.xinds_goaly:
			if not gmap.check_occupied_from_xy_index(ix, self.goaly, occupied_val=0.5):
				return False

		# all lower grid is occupied
		return True

	def update_turning_window(self):
		self.turing_window = [
			(self.moving_direction, 0.0),
			(self.moving_direction, self.sweep_direction),
			(0, self.sweep_direction),
			(-self.moving_direction, self.sweep_direction),
		]

	def swap_moving_direction(self):
		self.moving_direction *= -1
		self.update_turning_window()

	def search_start_grid(self, grid_map):
		xinds = []
		y_ind = 0
		if self.sweep_direction == self.SweepDirection.DOWN:
			xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
		elif self.sweep_direction == self.SweepDirection.UP:
			xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=False)

		if self.moving_direction == self.MovingDirection.RIGHT:
			return min(xinds), y_ind
		elif self.moving_direction == self.MovingDirection.LEFT:
			return max(xinds), y_ind

		raise ValueError("self.moving direction is invalid ")


def find_sweep_direction_and_start_posi(ox, oy):
	# find sweep_direction
	max_dist = 0.0
	vec = [0.0, 0.0]
	sweep_start_pos = [0.0, 0.0]
	for i in range(len(ox) - 1):
		dx = ox[i + 1] - ox[i]
		dy = oy[i + 1] - oy[i]
		d = np.hypot(dx, dy)

		if d > max_dist:
			max_dist = d
			vec = [dx, dy]
			sweep_start_pos = [ox[i], oy[i]]

	return vec, sweep_start_pos


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi):
	tx = [ix - sweep_start_posi[0] for ix in ox]
	ty = [iy - sweep_start_posi[1] for iy in oy]

	th = math.atan2(sweep_vec[1], sweep_vec[0])

	c = np.cos(-th)
	s = np.sin(-th)

	rx = [ix * c - iy * s for (ix, iy) in zip(tx, ty)]
	ry = [ix * s + iy * c for (ix, iy) in zip(tx, ty)]

	return rx, ry


def convert_global_coordinate(x, y, sweep_vec, sweep_start_posi):
	th = math.atan2(sweep_vec[1], sweep_vec[0])
	c = np.cos(th)
	s = np.sin(th)

	tx = [ix * c - iy * s for (ix, iy) in zip(x, y)]
	ty = [ix * s + iy * c for (ix, iy) in zip(x, y)]

	rx = [ix + sweep_start_posi[0] for ix in tx]
	ry = [iy + sweep_start_posi[1] for iy in ty]

	return rx, ry


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
	yind = None
	xinds = []

	if from_upper:
		xrange = range(grid_map.height)[::-1]
		yrange = range(grid_map.width)[::-1]
	else:
		xrange = range(grid_map.height)
		yrange = range(grid_map.width)

	for iy in xrange:
		for ix in yrange:
			if not grid_map.check_occupied_from_xy_index(ix, iy):
				yind = iy
				xinds.append(ix)
		if yind:
			break

	return xinds, yind


def setup_grid_map(ox, oy, reso, sweep_direction, offset_grid=20):
	width = math.ceil((max(ox) - min(ox)) / reso) + offset_grid
	height = math.ceil((max(oy) - min(oy)) / reso) + offset_grid
	center_x = np.mean(ox)
	center_y = np.mean(oy)

	grid_map = GridMap(width, height, reso, center_x, center_y)

	grid_map.set_value_from_polygon(ox, oy, 2.0, inside=False)

	grid_map.expand_grid()

	xinds_goaly = []
	goaly = 0
	if sweep_direction == SweepSearcher.SweepDirection.UP:
		xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
	elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
		xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=False)

	return grid_map, xinds_goaly, goaly

def sweep_path_search(sweep_searcher, gmap, grid_search_animation=False):
	# search start grid
	cxind, cyind = sweep_searcher.search_start_grid(gmap)
	if not gmap.set_value_from_xy_index(cxind, cyind, 0.5):
		print("Cannot find start grid")
		return [], []

	x, y = gmap.calc_grid_central_xy_position_from_xy_index(cxind, cyind)
	px, py = [x], [y]

	if grid_search_animation:
		fig, ax = plt.subplots()
		# for stopping simulation with the esc key.
		fig.canvas.mpl_connect('key_release_event',
				lambda event: [exit(0) if event.key == 'escape' else None])

	while True:
		cxind, cyind = sweep_searcher.move_target_grid(cxind, cyind, gmap)

		if sweep_searcher.is_search_done(gmap) or (cxind is None or cyind is None):
			print("Done")
			break

		x, y = gmap.calc_grid_central_xy_position_from_xy_index(
			cxind, cyind)

		px.append(x)
		py.append(y)

		gmap.set_value_from_xy_index(cxind, cyind, 0.5)

		if grid_search_animation:
			gmap.plot_grid_map(ax=ax)
			plt.pause(1.0)

	gmap.plot_grid_map()

	return px, py

def planning(ox, oy, reso,
			 moving_direction=SweepSearcher.MovingDirection.RIGHT,
			 sweeping_direction=SweepSearcher.SweepDirection.UP,
			 ):
	global rx, ry
	sweep_vec, sweep_start_posi = find_sweep_direction_and_start_posi(ox, oy)

	rox, roy = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi)

	gmap, xinds_goaly, goaly = setup_grid_map(rox, roy, reso, sweeping_direction)

	sweep_searcher = SweepSearcher(moving_direction, sweeping_direction, xinds_goaly, goaly)

	px, py = sweep_path_search(sweep_searcher, gmap)

	rx, ry = convert_global_coordinate(px, py, sweep_vec, sweep_start_posi)
	rl = len(rx)
	print("Path length:", rl)

	return rx, ry, rl


def planning_animation(ox, oy, reso):  # pragma: no cover
	px, py, pl = planning(ox, oy, reso)

	plt.cla()
	plt.plot(ox, oy, "-xb")
	plt.plot(px, py, "-r")
	plt.axis("equal")
	plt.grid(True)
	plt.pause(0.1)

	return px, py, pl

def simulate(poly):
	obstacles = []
	# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
	state = np.array([0, 0.2, np.pi/2, 0.0, 0.0])
	traj = state[:2]
	params = Params()
	plt.figure(figsize=(10,10))

	flight_area_vertices = poly

	gridmap = gm(flight_area_vertices, state[:2])
	
	ox = flight_area_vertices[:,0].tolist() + [flight_area_vertices[0,0]]
	oy = flight_area_vertices[:,1].tolist() + [flight_area_vertices[0,1]]

	goal_x = rx
	goal_y = ry
	# goal = [x, y], m
	goali = 0
	goal = [goal_x[goali], goal_y[goali]]
	t_prev_goal = time.time()

	# while True:
	for _ in range(params.numiters):
		state = motion(state, goal, params)

		goal_dist = np.linalg.norm(goal - state[:2])
		# print('Distance to goal %.2f [m]:' %goal_dist)
		t_current = time.time()
		if goal_dist < params.goal_tol or (t_current - t_prev_goal) > params.time_to_switch_goal: # goal is reached
			print('Switching to the next goal.')
			print('Time from the previous reached goal:', t_current - t_prev_goal)
			if goali < len(goal_x) - 1:
				goali += 1
			else:
				break
			t_prev_goal = time.time()
			goal = [goal_x[goali], goal_y[goali]]


		traj = np.vstack([traj, state[:2]])
		
		if params.animate:
			plt.cla()
			gridmap.draw_map(obstacles)
			plt.plot(goal_x, goal_y)
			plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal position', zorder=20)
			visualize(traj, state, params)
			plt.pause(0.1)

	print('Mission is complete!')
	plt.plot(goal_x, goal_y)
	visualize(traj, state, params)
	plt.show()