chessboard = [
[0,0,0,0,0,15,11,0],
[17,0,0,0,0,16,16,16],
[0,0,14,0,0,0,0,0],
[0,0,0,16,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,5,0,0,0,0],
[0,0,0,6,0,6,6,6],
[0,0,0,0,0,0,1,0]
]

"""
1 - white_king
2 - white_queen
3 - white_bishop
4 - white_knight
5 - white_rook
6 - white_pawn

11 - black_king
12 - black_queen
13 - black_bishop
14 - black_knight
15 - black_rook   17 - my rook
16 - black_pawn
"""


black_loc = [
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0]]

white_loc = [
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0]]

# King,queen,bishops,knights,rooks,pawns
black_positions = [[],[],[],[],[],[]]
white_positions = [[],[],[],[],[],[]]

# without any order, just the locations
Filled_white_locations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Filled_black_locations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


# append items into arrays with predefined dimensions
def append_front(lst,item):
    length = len(lst)
    for i in range(length-1):
        if lst[length -2 -i]:
            lst[length -1 -i] = lst[length -2 -i]
    lst[0] = item
    return lst


# making useful arrays of the chessboard locations
for i in range(8):
    for j in range(8):
        value = chessboard[i][j]
        if value>10:
            black_loc[i][j] = chessboard[i][j]
            append_front(Filled_black_locations, [i,j])

            val2 = value-10
            if val2 == 7:
                val2 = 5
            black_positions[val2-1].append([i,j])
        elif value:
            white_loc[i][j] = chessboard[i][j]
            white_positions[value - 1].append([i, j])
            append_front(Filled_white_locations, [i,j])


# To print the chessboard
def print_board(board):
    for i in range(8):
        print(board[i])


print("Black")
print_board(black_loc)
print("White")
print_board(white_loc)

print(black_positions)
print(white_positions)
print(Filled_black_locations)
print(Filled_white_locations)


# check whether the given position is on the chessboard
def is_on_board(pos):
    if pos[0]>7 or pos[1]>7 or pos[0]<0 or pos[1]<0:
        return False
    else:
        return True


# check whether the given position is blank
def is_idle(pos):
    if not chessboard[pos[0]][pos[1]]:
        return True
    else:
        return False


# check whether the given position has a white piece
def is_white_located(pos):
    if white_loc[pos[0]][pos[1]]:
        return True
    else:
        return False


# check whether the given position has a black piece
def is_black_located(pos):
    if black_loc[pos[0]][pos[1]]:
        return True
    else:
        return False


# possible white knight moves
def wknight_move(pos):
    pos_list = [[pos[0] + 1, pos[1] + 2],
                [pos[0] + 1, pos[1] - 2],
                [pos[0] + 2, pos[1] - 1],
                [pos[0] + 2, pos[1] + 1],
                [pos[0] - 1, pos[1] + 2],
                [pos[0] - 1, pos[1] - 2],
                [pos[0] - 2, pos[1] - 1],
                [pos[0] - 2, pos[1] + 1]]

    for i in range(8):
        if not is_on_board(pos_list[i]):
            pos_list[i] = False
        if pos_list[i] in Filled_white_locations:
            pos_list[i] = False

    return pos_list


# possible white king moves
def wking_move(pos):
    pos_list = [[pos[0], pos[1] - 1],
                [pos[0], pos[1] + 1],
                [pos[0] + 1, pos[1] - 1],
                [pos[0] + 1, pos[1] + 1],
                [pos[0] + 1, pos[1]],
                [pos[0] - 1, pos[1]],
                [pos[0] - 1, pos[1] - 1],
                [pos[0] - 1, pos[1] + 1]]
    for i in range(8):
        if not is_on_board(pos_list[i]):
            pos_list[i] = False
        if pos_list[i] in Filled_white_locations:
            pos_list[i] = False
    return pos_list


# possible white pawn moves
def wpawn_move(pos):  # checking locations that a white pawn can move (go straight if )
    if pos[0] == 6:
        pos_list = [[pos[0] - 1, pos[1] - 1],
                    [pos[0] - 1, pos[1] + 1],
                    [pos[0] - 1, pos[1]],
                    [pos[0] - 2, pos[1]]]
    else:
        pos_list = [[pos[0] - 1, pos[1] - 1],
                    [pos[0] - 1, pos[1] + 1],
                    [pos[0] - 1, pos[1]]]
    a = len(pos_list)
    for i in range(a):
        if not is_on_board(pos_list[i]):
            pos_list[i] = False
        if pos_list[i] in Filled_white_locations:
            pos_list[i] = False
        elif i == 0 or i == 1:
            if not pos_list[i] in Filled_black_locations:
                pos_list[i] = False
        elif i == 2:
            if pos_list[i] in Filled_black_locations:
                pos_list[i] = False
        else:
            if not pos_list[i-1] in Filled_black_locations:
                if not pos_list[i-1] in Filled_white_locations:
                    if not pos_list[i] in Filled_white_locations:
                        if not pos_list[i] in Filled_white_locations:
                            pass
                        else:
                            pos_list[i] = False
                    else:
                        pos_list[i] = False
                else:
                    pos_list[i] = False
            else:
                pos_list[i] = False



    return pos_list


def wrook_move(pos):
    pos_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    count = 0
    for i in range(8):
        if i != pos[0]:
            pos_list[count] = [i,pos[1]]
            count += 1

    for j in range(8):
        if j != pos[1]:
            pos_list[count] = [pos[0],j]
            count += 1

    # chech whether the position can be obtained



    return pos_list





wkingpos = white_positions[0][0]
myrookpos = [1, 0]  # a7

# two positions that my rook can take to check mate the white king
my_rook_move = [[wkingpos[0], myrookpos[1]], [wkingpos[1], myrookpos[0]]]

print(wrook_move([6,7]))









