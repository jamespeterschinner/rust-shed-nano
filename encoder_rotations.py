from itertools import tee, islice, cycle


def nwise(xs, n=2): return zip(*(islice(xs, idx, None)
                                 for idx, xs in enumerate(tee(xs, n))))


def take(n, iterable):
    return list(islice(iterable, n))


rotations = [3, 2, 0, 1]


def generate_valid_rotation(rotations):
    return '{0:#x}'.format(int(f''.join(['{0:0=2b}{1:0=2b}'.format(a, b) for (a, b) in list(nwise(take(4, rotations)))]), 2))


ccw_rotations = [generate_valid_rotation(
    islice(cycle(rotations), i, None)) for i in range(0, 4)]

cw_rotations = [generate_valid_rotation(
    islice(cycle(reversed(rotations)), i, None)) for i in range(0, 4)]
