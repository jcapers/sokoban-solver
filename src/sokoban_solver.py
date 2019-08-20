from enum import Enum
from typing import List, Tuple
import heapq
import sys
import time

# Internal package imports
from state import State
from sokoban_map import SokobanMap

"""
COMP3702 Assignment 1: Sokoban Solver

Implementation of UCS and A* based on [1] and [2].

Freeze Deadlock implementation based on [3] though not all deadlock solutions have
been implemented on this page (e.g., have not implemented Simple Deadlocks).

References
[1] S. Russell, P. Norvig, Artificial Intelligence: A Modern Approach, 3rd ed. 
    Essex, England: Pearson Education, 2016
[2] A. Bialkowski, Lecture, Topic: "Search in Discrete Space." COMP3702, 
    ITEE-EAIT, University of Queensland. Aug, 2019.
[3] How to detect deadlocks - Sokoban Wiki. Accessed Sept. 09. 2019. 
    [Online] Available: http://sokobano.de/wiki/index.php?title=How_to_detect_deadlocks
"""


class SearchStrategy(Enum):
    UCS = 1
    AStar = 2


class SokobanSolver:
    """Sokoban Solver, utilises Uniform Cost Search OR A* Search."""

    def __init__(self, input_filename: str, output_filename: str, search_strategy: str = 'ASTAR'):
        self.map_file = input_filename
        self.output_file = output_filename

        # Load the initial state of a Sokoban game, keep track of target and obstacles in sokoban_map
        self.sokoban_map = SokobanMap(self.map_file)
        self.moves = {'LEFT': 'l',
                      'RIGHT': 'r',
                      'UP': 'u',
                      'DOWN': 'd'}

        # Setup for search
        self.search_strategy = search_strategy
        self.initial_state = State(None, self.sokoban_map.box_positions[:], self.sokoban_map.player_position)
        self.frontier = []
        self.visited = {}
        self.nodes_generated = 0

    def solve_sokoban(self):
        """
        Run the Sokoban Solver and process outputs to stdout and path solution to output_file.
        :return:
        """
        if self.search_strategy == 'UCS':
            # Do UCS
            print('Started UCS Search...')
            start = time.time()
            final_state = self.search(SearchStrategy.UCS)
        else:
            # DO ASTAR
            print('Started A* Search...')
            start = time.time()
            final_state = self.search(SearchStrategy.AStar)

        if final_state is None:
            print('Failed to find solution...')
            return
        else:
            print('\nFinding path...')
            path = self.find_path(final_state)
            end = time.time()

        # Output Path
        print(f'Path ({len(path)}) steps: {path}')
        output_path = f'{self.output_file}_{self.search_strategy}_path.txt'
        with open(output_path, 'w+') as f:
            f.write(",".join(path))

        # Output Stats
        print(f'Nodes Gen.: {self.nodes_generated}, Fringe: {len(self.frontier)}, Explored: {len(self.visited)}, Time: {round(end-start, 4)} secs')
        stats_path = f'{self.output_file}_{self.search_strategy}_stats.txt'
        with open(stats_path, 'w+') as f:
            f.write(f'Path ({len(path)} steps): {",".join(path)}\n')
            f.write(f'Number of nodes generated: {self.nodes_generated}\n')
            f.write(f'Number of frontier nodes left: {len(self.frontier)}\n')
            f.write(f'Number of visited nodes: {len(self.visited)}\n')
            f.write(f'{self.search_strategy} Time: {end - start} secs\n')
        print(f'Path saved to {output_path}, Stats to {stats_path}')

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
        self.frontier.clear()
        self.visited.clear()
        self.nodes_generated = 0
        # Initialise min priority queue with the starting state, and reset.
        heapq.heappush(self.frontier, (self.initial_state.cost, self.initial_state))
        # costs_table = {}

        while self.frontier:
            # Get state, if not goal find new states.
            cost, current_state = heapq.heappop(self.frontier)
            if self.goal_test(current_state):
                return current_state
            self.visited[current_state] = current_state
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
                if not self.visited.get(state, False):
                    # old_cost = costs_table.get(state, -10000)
                    # if f_cost < old_cost:
                    #     self.frontier.remove((old_cost, state))
                    #     heapq.heapify(self.frontier)
                # if state not in self.visited:
                    for old_cost, old_state in self.frontier:
                        # Remove same state of higher cost.
                        if state == old_state and f_cost <= old_cost:
                            self.frontier.remove((old_cost, old_state))
                            heapq.heapify(self.frontier)
                            break
                    heapq.heappush(self.frontier, (f_cost, state))
                    # costs_table[state] = f_cost
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
        1) Distance player to closest box.
           -- This is the min manhattan distance a player must move to attempt
           to push a box.
        2) Distance each box is from nearest goal.
           -- This is the min manhattan distance a box must be pushed to reach
           a goal.
           -- If a box is already on a goal, then the heuristic will be smaller
           to reflect closeness to final goal state.
        :param state: state to calculate a heuristic for.
        :return: cost for the given state.
        """
        boxes = state.box_positions
        player = state.player_position
        goals = self.sokoban_map.tgt_positions
        h_cost = 0

        # Player
        player_costs = [self.manhattan_distance(player, box) for box in boxes]
        h_cost += max(player_costs)

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

        Criteria:
        1) Check Obstacles, player cannot move into obstacles
        2) If a box is in the new position, check if the box can be moved.
            -- Box cannot be moved if obstacle or another box is in the way.
        3) Move must not send a box into a deadlock state (at least the deadlocks
           that are currently detectable).

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

        # If a box has been moved, check if the box is now deadlocked, okay if it's in a goal position.
        if box_moved and (new_box_y, new_box_x) not in self.sokoban_map.tgt_positions:
            boxes = state.box_positions[:]
            boxes.remove((new_y, new_x))
            boxes.append((new_box_y, new_box_x))
            if self.deadlock_test(new_box_y, new_box_x, boxes):
                return False
        # Create new state
        new_box_positions = state.box_positions[:]
        if box_moved:
            new_box_positions.remove((new_y, new_x))
            new_box_positions.append((new_box_y, new_box_x))
        return State(state, new_box_positions, (new_y, new_x), move)

    def deadlock_test(self, box_y, box_x, boxes):
        """
        Test if a deadlock exists with box position.
        :param box_y: y coordinate for box.
        :param box_x: x coordinate for box.
        :param boxes: list of box positions in proposed new state.
        :return: True if deadlock detected.
        """
        # Prepare some variables for ease
        goals = self.sokoban_map.tgt_positions[:]
        obstacles = self.sokoban_map.obstacle_map[:]

        x_edges = (1, self.sokoban_map.x_size - 2)
        y_edges = (1, self.sokoban_map.y_size - 2)

        # Check all deadlock cases
        if box_y in y_edges or box_x in x_edges:
            # Special deadlock case if box is against edge of map.
            if self.deadlock_map_edge(box_y, box_x, x_edges, y_edges, goals, obstacles, boxes):
                return True
        # Freeze deadlock can be checked now, as edge case passed.
        elif self.deadlock_freeze_check(box_y, box_x, obstacles, boxes):
            return True

        return False

    def deadlock_map_edge(self, box_y, box_x, x_edges, y_edges, goals, obstacles, boxes):
        """
        Simple check to see if a box can reach the target from the edge of map, if not
        then it will never be pushable off the wall so will fail.

        1) If a box is against the walls, then it can only move to targets also against the walls
           on the strict edges of the map (e.g., a square map).

        :param box_y: y coordinate of box.
        :param box_x: x coordinate of box.
        :param x_edges: general x positions at edge of map that is traversable.
        :param y_edges: general y positions at edge of map that is traversable.
        :param goals: list containing goal/target coordinates.
        :param obstacles: list containing obstacle coordinates.
        :param boxes: list of boxes in proposed new state.
        :return: True if deadlocked, else False.
        """
        goals_x = [x for y, x in goals]
        goals_y = [y for y, x in goals]
        # If the box is against a wall of the map then it can only traverse along this wall.
        # Check sides
        if box_x == x_edges[0] or box_x == x_edges[1]:
            # Goal not on same side/column, impossible to reach goal.
            if box_x not in goals_x:
                return True
            elif self.blocked_vertical(box_y, box_x, obstacles) or \
                    (box_y - 1, box_x) in boxes or \
                    (box_y + 1, box_x) in boxes:
                return True

        # Check top/bottom
        if box_y == y_edges[0] or box_y == y_edges[1]:
            if box_y not in goals_y:
                return True
            elif self.blocked_horizontal(box_y, box_x, obstacles) or \
                    (box_y, box_x - 1) in boxes or \
                    (box_y, box_x + 1) in boxes:
                return True

        # Safe!
        return False

    def deadlock_freeze_check(self, box_y, box_x, obstacles, boxes):
        """
        Checks freeze deadlock case [3].

        Uses blocked_horizontal and blocked_vertical to check blockages.

        If a box is detected, convert current box into a wall and check if blocking box is blocked. The wall conversion
        for the temporary obstacle list is required to avoid circular checks.

        This function makes use of the provided sokoban_map.py script/classes.

        :param box_y: y position of box.
        :param box_x: x position of box.
        :param obstacles: obstacle coordinates on current sokoban map.
        :param boxes: list of boxes in proposed new state.
        :return: True if a freeze deadlock is detected.
        """
        # Deep copy of list in case we need to add boxes.
        obstacles_check_list = obstacles
        wall = self.sokoban_map.OBSTACLE_SYMBOL
        # Check horizontal directions.
        if self.blocked_horizontal(box_y, box_x, obstacles_check_list):
            # If one horizontal direction is blocked by a wall, check vertical.
            if self.blocked_vertical(box_y, box_x, obstacles_check_list):
                # The box can't move since two directions are blocked by obstacles.
                return True
            # If vertical is not blocked, check if a box is in a vertical space (and not temporarily treated as a wall)
            # elif (box_y - 1, box_x) in boxes:
            #     # Treat THIS box as a standard obstacle to avoid circular checks.
            #     obstacles_check_list[box_y][box_x] = wall
            #     if self.deadlock_freeze_check(box_y - 1, box_x, obstacles_check_list, boxes):
            #         return True
            # elif (box_y + 1, box_x) in boxes:
            #     obstacles_check_list[box_y][box_x] = wall
            #     if self.deadlock_freeze_check(box_y + 1, box_x, obstacles_check_list, boxes):
            #         return True

        # Check vertical directions, similar to horizontal check but reversed.
        if self.blocked_vertical(box_y, box_x, obstacles_check_list):
            if self.blocked_horizontal(box_y, box_x, obstacles_check_list):
                return True
            # elif (box_y, box_x - 1) in boxes:
            #     obstacles_check_list[box_y][box_x] = wall
            #     if self.deadlock_freeze_check(box_y, box_x - 1, obstacles_check_list, boxes):
            #         return True
            # elif (box_y, box_x + 1) in boxes:
            #     obstacles_check_list[box_y][box_x] = wall
            #     if self.deadlock_freeze_check(box_y, box_x + 1, obstacles_check_list, boxes):
            #         return True

        # Got here, so not blocked.
        return False

    def blocked_horizontal(self, box_y, box_x, obstacles):
        """
        Checks if box is blocked horizontally by obstacles.

        Obstacles are generally walls, but can be boxes if box
        already checked in deadlocks.

        :param box_y: y coordinate of box.
        :param box_x: x coordinate of box.
        :param obstacles: obstacle coordinates of current sokoban map.
        :return: True if box is blocked horizontally (x-axis).
        """
        wall = self.sokoban_map.OBSTACLE_SYMBOL
        # Check horizontal directions.
        if obstacles[box_y][box_x - 1] == wall or obstacles[box_y][box_x + 1] == wall:
            return True
        return False

    def blocked_vertical(self, box_y, box_x, obstacles):
        """
        Checks if box is blocked vertically by obstacles.

        Obstacles are generally walls, but can be boxes if box
        already checked in deadlocks.

        :param box_y: y coordinate of box.
        :param box_x: x coordinate of box.
        :param obstacles: obstacle coordinates of current sokoban map.
        :return: True if box is blocked vertically (y-axis).
        """
        wall = self.sokoban_map.OBSTACLE_SYMBOL
        # Check vertical directions.
        if obstacles[box_y - 1][box_x] == wall or obstacles[box_y + 1][box_x] == wall:
            return True
        return False


def main(arglist: List[str]):
    if len(arglist) < 2:
        print("Sokoban Solver needs an input file and an output file name.")
        print("Usage: sokoban_solver.py "
              "[map_file_name] [solution_file_name] [optional strategy = 'UCS' or 'ASTAR']")
        return
    elif len(arglist) == 2:
        input_file = arglist[0]
        output_file = arglist[1]
        sokoban_solver = SokobanSolver(input_file, output_file)
    elif len(arglist) == 3:
        input_file = arglist[0]
        output_file = arglist[1]
        search_strategy = str(arglist[2]).upper()
        sokoban_solver = SokobanSolver(input_file, output_file, search_strategy)
    sokoban_solver.solve_sokoban()


if __name__ == '__main__':
    main(sys.argv[1:])
