#!/usr/bin/env python
# vim:fileencoding=utf8
#
# Project: Implementation of the Lemke-Howson algorithm for finding MNE
# Author:  Petr Zemek <s3rvac@gmail.com>, 2009
#

"""Runs a program which computes MNE in the given 2-player game
using the Lemke-Howson algorithm.
"""


import sys
import math
import re

def lhmne(inputMatrix):
# def main():
    try:
        # These imports must be here because of possible
        # SyntaxError exceptions in different versions of python
        # (this program needs python 3.5)
        import output
        import lh

        # Check program arguments (there should be none)
        if len(sys.argv) > 1:
            stream = sys.stderr
            if sys.argv[1] in ['-h', '--help']:
                stream = sys.stdout
            output.printHelp(stream)
            return 1

        # Obtain input matrices from the standard input
        # m1, m2 = src.io.parseInputMatrices(sys.stdin.read())
        m1, m2 = output.parseInputMatrices(inputMatrix)

        # Compute the equilibirum
        eq = lh.lemkeHowson(m1, m2)

        result = re.findall(r"\d+\/?\d*", output.printEquilibrium(eq, sys.stdout))

        # # Print both matrices and the result
        # src.io.printGameInfo(m1, m2, eq, sys.stdout)

        return result
    except SyntaxError:
        sys.stderr.write('Need python 3.5 to run this program.\n')
    except Exception as e:
        sys.stderr.write('Error: ' + e.message + '\n')
        return 1


# if __name__ == '__main__':
#     main()
