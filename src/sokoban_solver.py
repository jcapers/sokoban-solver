from enum import Enum
from typing import List
import heapq
import sys

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

        # Setup for search
        self.initial_state = State(None, self.sokoban_map.box_positions[:], self.sokoban_map.player_position)
        self.frontier = [self.initial_state]
        self.visited = []

    def search(self, strategy: SearchStrategy):
        """
        Search for solution to Sokoban game.

        If strategy is A* then will incorporate heuristic function.

        Algorithm modified but based on pseudocode provided by Russsell and Norvig [1]
        and Alina Bialkowski's lecture notes [2].

        :param strategy:
        :return:
        """
        pass

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


def main(arglist: List[str]):
    if len(arglist) != 2:
        print("Sokoban Solver needs an input file and an output file name.")
        print("Usage: sokoban_solver.py [map_file_name] [solution_file_name]")
        return
    input_file = arglist[0]
    output_file = arglist[1]
    sokoban_solver = SokobanSolver(input_file, output_file)


if __name__ == '__main__':
    main(sys.argv[1:])
