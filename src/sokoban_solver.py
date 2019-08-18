from enum import Enum
from typing import List, Tuple
import heapq
import os
import sys
import time

# Internal package imports
from state import State
from sokoban_map import SokobanMap

"""
COMP3702 Assignment 1: Sokoban Solver

Implementation of UCS and A* based on [1] and [2].

Deadlock implementation based on [3] and [4] though not all deadlock solutions have
been implemented in this solution.

References
[1]
[2]
[3]
[4]
"""


class SearchStrategy(Enum):
    UCS = 1
    AStar = 2


class SokobanSolver:
    """Sokoban Solver, utilises Uniform Cost Search OR A* Search."""

    def __init__(self, input_filename: str, output_filename: str):
        self.map_file = input_filename
        self.output_file = output_filename

        # Load the initial state of a Sokoban game, keep track of target and obstacles in sokoban_map
        self.sokoban_map = SokobanMap(self.map_file)
        self.moves = {'LEFT': 'l',
                      'RIGHT': 'r',
                      'UP': 'u',
                      'DOWN': 'd'}

        # Setup for search
        self.initial_state = State(None, self.sokoban_map.box_positions[:], self.sokoban_map.player_position)
        self.frontier = []
        self.visited = []
        self.nodes_generated = 0

    def solve_sokoban(self):
        """
        Run the Sokoban Solver and process outputs to stdout and path solution to output_file.
        :return:
        """
        # UCS Search from until path generated.
        print(f'Started UCS Search...')
        self.frontier.clear()
        self.visited.clear()
        self.nodes_generated = 0
        start = time.time()
        final_state = self.search(SearchStrategy.UCS)
        if final_state is None:
            print('Failed to find solution...')
            return
        print('\nFinding path...')
        path = self.find_path(final_state)
        end = time.time()

        # Output
        ucs_output_path = self.output_file + '_ucs_path.txt'
        with open(ucs_output_path, 'w+') as f:
            f.write(",".join(path))
        ucs_stats_path = self.output_file + '_ucs_stats.txt'
        with open(ucs_stats_path, 'w+') as f:
            f.write(f'UCS Took: {end - start} secs\n')
            f.write(f'Path ({len(path)} steps): {",".join(path)}\n')
            f.write(f'Number of nodes generated: {self.nodes_generated}\n')
            f.write(f'Number of frontier nodes left: {len(self.frontier)}\n')
            f.write(f'Number of visited nodes: {len(self.visited)}\n')
        print(f'Path saved to {ucs_output_path}')

        # AStar search until path generated.
        print(f'Started A* Search...')
        self.frontier.clear()
        self.visited.clear()
        self.nodes_generated = 0
        start = time.time()
        final_state = self.search(SearchStrategy.AStar)
        if final_state is None:
            print('Failed to find state...')
            return
        print('\nFinding path...')
        path = self.find_path(final_state)
        end = time.time()

        # Output
        astar_output_path = self.output_file + '_astar_path.txt'
        with open(astar_output_path, 'w+') as f:
            f.write(",".join(path))
        astar_stats_path = self.output_file + '_astar_stats.txt'
        with open(astar_stats_path, 'w+') as f:
            f.write(f'A* Took: {end - start} secs\n')
            f.write(f'Path ({len(path)} steps): {",".join(path)}\n')
            f.write(f'Number of nodes generated: {self.nodes_generated}\n')
            f.write(f'Number of frontier nodes left: {len(self.frontier)}\n')
            f.write(f'Number of visited nodes: {len(self.visited)}\n')
        print(f'Path saved to {astar_output_path}')

    def find_path(self, final_state):
        """
        Finds path after solver has found a solution.
        :return: list of moves taken to get to goal.
        """
        path = []
        current_node = final_state
        # Parent does not have any 'moves' so stop there.
        while current_node.move_taken:
            path.append(current_node.move_taken)
            current_node = current_node.parent
        path.reverse()
        return path

    def search(self, strategy: SearchStrategy):
        """
        Search for solution to Sokoban game.

        If strategy is A* then will incorporate heuristic function, otherwise using Uniform Cost Search.

        Algorithm modified but based on pseudocode provided by Russsell and Norvig [1]
        and Alina Bialkowski's lecture notes [2].

        :param strategy: Search Strategy to apply (UCS or A*)
        :return: Goal state node, or none if failed.
        """
        # Initialise min priority queue with the starting state, and reset.
        heapq.heappush(self.frontier, (self.initial_state.cost, self.initial_state))

        while self.frontier:
            # Get state, if not goal find new states.
            cost, current_state = heapq.heappop(self.frontier)
            if self.goal_test(current_state):
                return current_state
            self.visited.append(current_state)
            next_states = self.next_states(current_state)
            self.nodes_generated += len(next_states)

            # Compute heuristic if A* and update cost
            if strategy is SearchStrategy.AStar:
                for state in next_states:
                    state.h_cost += self.heuristic(state)

            # Push into heap with heapq (cost, state)
            for state in next_states:
                f_cost = state.cost + state.h_cost
                # Only push to heap if not already visited and not already in heap.
                if state not in self.visited and (f_cost, state) not in self.frontier:
                    for old_cost, old_state in self.frontier:
                        # Remove same state of higher cost.
                        if state == old_state and f_cost < old_cost:
                            self.frontier.remove((old_cost, old_state))
                            heapq.heapify(self.frontier)
                            break
                    heapq.heappush(self.frontier, (f_cost, state))
        return None

    def goal_test(self, state):
        """
        Test if state is at goal by comparing box positions with target positions.

        :param state: the state to check if the goal has been reached.
        :return: True if state is the goal state, else False.
        """
        for box in state.box_positions:
            if box not in self.sokoban_map.tgt_positions:
                return False
        return True

    def heuristic(self, state):
        """
        Computes heuristic for state.

        Heuristic is calculated by:
        1) Distance player from all boxes.
        2) Distance each box is from nearest goal.
        :param state: state to calculate a heuristic for.
        :return: cost for the given state.
        """
        boxes = state.box_positions
        player = state.player_position
        goals = self.sokoban_map.tgt_positions
        h_cost = 0

        # Player
        player_costs = [self.manhattan_distance(player, box) for box in boxes]
        h_cost += min(player_costs)

        # Boxes
        for box in boxes:
            box_cost = [self.manhattan_distance(box, goal) for goal in goals]
            h_cost += min(box_cost)

        return h_cost

    def manhattan_distance(self, a, b):
        """
        Computes manhattan distance of two positions.
        :param a: Coordinate tuple, (y, x)
        :param b: Coordinate tuple to calculate distance to a, (y, x)
        :return: manhattan distance between a and b
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def next_states(self, state):
        """
        Computes the next valid states from state.

        :param state: The current state of the game.
        :return: list of valid state objects.
        """
        valid_states = []
        # Check each direction
        for move in self.moves.values():
            new_state = self.test_move(state, move)
            if new_state:
                valid_states.append(new_state)
        return valid_states

    def test_move(self, state, move):
        """
        Test if move is valid.

        1) Check Obstacles, player cannot move into obstacles
        2) If a box is in the new position, check if the box can be moved.
            -- Box cannot be moved if obstacle or another box is in the way.

        Modified from sokoban_map.apply_move.

        Deadlocks
        1) Check if a box can reach a target (e.g., if the box is along the top wall,
           it can only reach targets along the top wall)
        2) Check corners and sides, i.e,. box blocked on side and (top or bottom).
        2) Check when box is blocked vertically or horizontally, and also blocked once in other direction.

        :param state: Current state where move begins.
        :param move: Possible move, string representing move.
        :return: New state if valid move, else False.
        """
        # Check player movement
        player_y, player_x = state.player_position
        obstacle = self.sokoban_map.OBSTACLE_SYMBOL
        box_moved = False

        if move is self.moves['LEFT']:
            new_y, new_x = player_y, player_x - 1
            if self.sokoban_map.obstacle_map[new_y][new_x] == obstacle:
                return False
            if (new_y, new_x) in state.box_positions:
                if self.sokoban_map.obstacle_map[new_y][new_x - 1] == obstacle or (new_y, new_x - 1) in state.box_positions:
                    return False
                else:
                    new_box_y = new_y
                    new_box_x = new_x - 1
                    box_moved = True
        elif move is self.moves['RIGHT']:
            new_y, new_x = player_y, player_x + 1
            if self.sokoban_map.obstacle_map[new_y][new_x] == obstacle:
                return False
            if (new_y, new_x) in state.box_positions:
                if self.sokoban_map.obstacle_map[new_y][new_x + 1] == obstacle or (new_y, new_x + 1) in state.box_positions:
                    return False
                else:
                    new_box_y = new_y
                    new_box_x = new_x + 1
                    box_moved = True
        elif move is self.moves['UP']:
            new_y, new_x = player_y - 1, player_x
            if self.sokoban_map.obstacle_map[new_y][new_x] == obstacle:
                return False
            if (new_y, new_x) in state.box_positions:
                if self.sokoban_map.obstacle_map[new_y - 1][new_x] == obstacle or (new_y - 1, new_x) in state.box_positions:
                    return False
                else:
                    new_box_y = new_y - 1
                    new_box_x = new_x
                    box_moved = True
        else:
            # Down
            new_y, new_x = player_y + 1, player_x
            if self.sokoban_map.obstacle_map[new_y][new_x] == obstacle:
                return False
            if (new_y, new_x) in state.box_positions:
                if self.sokoban_map.obstacle_map[new_y + 1][new_x] == obstacle or (new_y + 1, new_x) in state.box_positions:
                    return False
                else:
                    new_box_y = new_y + 1
                    new_box_x = new_x
                    box_moved = True

        # If a box has been moved, check if the box is now deadlocked, okay if it's in a goal position.
        if box_moved and (new_box_y, new_box_x) not in self.sokoban_map.tgt_positions:
            if self.deadlock_test(new_box_y, new_box_x, state):
                return False
        # Create new state
        new_box_positions = state.box_positions[:]
        if box_moved:
            new_box_positions.remove((new_y, new_x))
            new_box_positions.append((new_box_y, new_box_x))
        return State(state, new_box_positions, (new_y, new_x), move)

    def deadlock_test(self, box_y, box_x, state):
        """
        Test if a deadlock exists with box position.
        :param box_y:
        :param box_x:
        :param state:
        :return:
        """
        # Prepare some variables for ease
        goals = self.sokoban_map.tgt_positions
        obstacles = self.sokoban_map.obstacle_map
        x_edges = (1, self.sokoban_map.x_size - 2)
        y_edges = (1, self.sokoban_map.y_size - 2)

        # Check all deadlock cases
        if box_y in y_edges or box_x in x_edges:
            # Special deadlock case if box is against edge of map.
            if self.deadlock_wall_check(box_y, box_x, x_edges, y_edges, goals, obstacles, state):
                return True
        # Freeze deadlock can be checked now, as edge case passed.
        elif self.deadlock_freeze_check(box_y, box_x, obstacles, state):
            return True

        return False

    def deadlock_wall_check(self, box_y, box_x, x_edges, y_edges, goals, obstacles, state):
        """
        Deadlock checking based on sokobano wiki [3][4].
        Simple check to see if a box can reach the target from the walls.

        1) If a box is against the walls, then it can only move to targets also against the walls.
        2) If the box is against a wall then it may be blocked by other obstacles.
           -- Note for now this just checks against obstacles/walls, not other boxes.

        :return: True if deadlocked, else False.
        """
        goals_x = [x for y, x in goals]
        goals_y = [y for y, x in goals]
        # If the box is against a wall of the map then it can only traverse along this wall.
        # Check sides
        if box_x == x_edges[0] or box_x == x_edges[1]:
            if box_x not in goals_x:
                return True
            elif (box_y - 1, box_x) in (obstacles or state.box_positions) \
                    or (box_y + 1, box_x) in (obstacles or state.box_positions):
                return True

        # Check top/bottom
        if box_y == y_edges[0] or box_y == y_edges[1]:
            if box_y not in goals_y:
                return True
            elif (box_y, box_x - 1) in (obstacles or state.box_positions) \
                    or (box_y, box_x + 1) in (obstacles or state.box_positions):
                return True

        # Safe!
        return False

    def deadlock_freeze_check(self, box_y, box_x, obstacles, state):
        """
        Checks freeze deadlock case [3] [4].

        Uses deadlock_horizontal_check and deadlock_vertical_check.

        :param box_y:
        :param box_x:
        :param obstacles:
        :param state:
        :return:
        """
        # Deep copy of list in case we need to add boxes.
        obstacles_check_list = obstacles[:]
        # Check horizontal directions.

        # If one horizontal direction is blocked by a wall, check vertical.

            # If vertical is blocked, return true.

            # If vertical is not blocked, check if a box is in a vertical space (and not temporarily treated as a wall)

                # If yes, run the check vertical test but add this box into obstacles to be treated as a wall, return true if this subsequent test passes.

        # Check vertical directions.

            # Repeat as above

        # Got here, so not blocked.
        return False

    def deadlock_horizontal_check(self, box_y, box_x, obstacles):
        """
        Checks horizontal deadlock case where box is blocked horizontally, and
        might be blocked in at least one vertical direction, hence causing a
        deadlock.

        :param box_y:
        :param box_x:
        :param obstacles:
        :param state:
        :return:
        """
        # Check horizontal directions.

        # If one horizontal direction is blocked by a wall, check vertical.

        # If vertical is blocked, return true.

        return False

    def deadlock_vertical_check(self, box_y, box_x, obstacles):
        """
        Checks vertical deadlock case where box is blocked vertically, and may
        be blocked in at least one horizontal direction, hence causing a
        deadlock.

        :param box_y:
        :param box_x:
        :param obstacles:
        :param state:
        :return:
        """
        # Check vertical directions.

        # If one vertical direction is blocked by a wall, check horizontal.

        # If horizontal is blocked, return true.

        return False




def main(arglist: List[str]):
    if len(arglist) != 2:
        print("Sokoban Solver needs an input file and an output file name.")
        print("Usage: sokoban_solver.py [map_file_name] [solution_file_name]")
        return
    input_file = arglist[0]
    output_file = arglist[1]
    sokoban_solver = SokobanSolver(input_file, output_file)
    sokoban_solver.solve_sokoban()


if __name__ == '__main__':
    main(sys.argv[1:])
