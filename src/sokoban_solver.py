from enum import Enum
from typing import List
import heapq
import os
import sys
import time

# Internal package imports
from state import State
from sokoban_map import SokobanMap

"""
COMP3702 Assignment 1: Sokoban Solver

References
[1]
[2]
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
        print(f'Started UCS Search...')
        self.frontier.clear()
        self.visited.clear()
        self.nodes_generated = 0
        start = time.time()
        final_state = self.search(SearchStrategy.UCS)
        end = time.time()
        print(f'UCS Took: {end - start} secs')
        print(f'Result boxes: {final_state.box_positions}')
        print(f'Goal: {self.sokoban_map.tgt_positions}')
        print('\nFinding path...')
        path = self.find_path(final_state)
        ucs_output_path = self.output_file + '_ucs_path.txt'
        with open(ucs_output_path, 'w+') as f:
            f.write(",".join(path))
        ucs_stats_path = self.output_file + '_ucs_stats.txt'
        with open(ucs_stats_path, 'w+') as f:
            f.write(f'UCS Took: {end - start} secs\n')
            f.write(f'Path: {",".join(path)}\n')
            f.write(f'Number of nodes: {self.nodes_generated}\n')
            f.write(f'Number of frontier nodes left: {len(self.frontier)}\n')
            f.write(f'Number of visited nodes: {len(self.visited)}\n')
        print(f'Path saved to {ucs_output_path}')

        print(f'Started A* Search...')
        self.frontier.clear()
        self.visited.clear()
        self.nodes_generated = 0
        start = time.time()
        final_state = self.search(SearchStrategy.AStar)
        end = time.time()
        print(f'A* Took: {end - start} secs')
        print(f'Result boxes: {final_state.box_positions}')
        print(f'Goal: {self.sokoban_map.tgt_positions}')
        print('\nFinding path...')
        path = self.find_path(final_state)
        astar_output_path = self.output_file + '_astar_path.txt'
        with open(astar_output_path, 'w+') as f:
            f.write(",".join(path))
        astar_stats_path = self.output_file + '_astar_stats.txt'
        with open(astar_stats_path, 'w+') as f:
            f.write(f'A* Took: {end - start} secs\n')
            f.write(f'Path: {",".join(path)}\n')
            f.write(f'Number of nodes: {self.nodes_generated}\n')
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
                    state.cost += self.heuristic(state)

            # Push into heap with heapq (cost, state)
            for state in next_states:
                # Only push to heap if not already visited and not already in heap.
                if state not in self.visited and (state.cost, state) not in self.frontier:
                    for old_cost, old_state in self.frontier:
                        # Remove same state of higher cost.
                        if state == old_state and state.cost < old_cost:
                            self.frontier.remove((old_cost, old_state))
                            heapq.heapify(self.frontier)
                            break
                    heapq.heappush(self.frontier, (state.cost, state))
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
        1) Distance player is from nearest box.
        2) Distance each box is from nearest goal.
        :param state:
        :return:
        """
        boxes = state.box_positions
        player = state.player_position
        goals = self.sokoban_map.tgt_positions
        h_cost = 0

        # # Player
        # player_costs = [self.manhattan_distance(player, box) for box in boxes]
        # h_cost += min(player_costs)

        # Boxes
        for box in boxes:
            box_cost = [self.manhattan_distance(box, goal) for goal in goals]
            h_cost += min(box_cost)

        return h_cost

    def manhattan_distance(self, a, b):
        """
        Computes manhattan distance of two positions.
        :param state1:
        :param state2:
        :return:
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

        # Create new state
        new_box_positions = state.box_positions[:]
        if box_moved:
            new_box_positions.remove((new_y, new_x))
            new_box_positions.append((new_box_y, new_box_x))
        return State(state, new_box_positions, (new_y, new_x), move)


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
