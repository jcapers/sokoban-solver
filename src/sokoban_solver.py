from sokoban_map import SokobanMap
from typing import List
import sys


def main(args: List[str]):
    input_file = args[0]
    sokoban = SokobanMap(input_file)
    print(f'Obstacle Map\n{sokoban.obstacle_map}')
    print(f'Player Position\n{sokoban.player_position}')
    print(f'Target Positions\n{sokoban.tgt_positions}')
    print(f'Box Positions\n{sokoban.box_positions}')


if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
