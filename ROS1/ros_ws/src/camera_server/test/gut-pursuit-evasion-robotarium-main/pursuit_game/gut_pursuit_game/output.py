# vim:fileencoding=utf8
#
# Project: Implementation of the Lemke-Howson algorithm for finding MNE
# Author:  Petr Zemek <s3rvac@gmail.com>, 2009
#

"""I/O functions "communicating" with the user of the program."""


import os

import matrix


def parseInputMatrices(text):
    """Parses two matrices from the selected text and returns
    them in a tuple (m1, m2).

    text - text from which the matrices will be parsed (string)

    Preconditions:
        - text must contain two matrices (see matrix.Matrix.__repr__())
          separated by an extra new line (there might be additional
          newlines after the second matrix, which are ignored)
        - both matrices must have the same number of rows and cols

    Raises ValueError if some of the preconditions are not met.
    """
    try:
        mTexts = text
        # mTexts = text.split('\n\n')
        if len(mTexts) > 2:
            # Check whether there are only newlines in the rest of the text
            for mText in mTexts[2:]:
                for c in mText:
                    if c != '\n':
                        raise ValueError('Redundant characters at the end ' +\
                                'of the input.')
            mTexts = (mTexts[0], mTexts[1] + '\n')

        m1 = matrix.fromText(mTexts[0] + '\n')
        m2 = matrix.fromText(mTexts[1])
    except IndexError:
        raise ValueError('Input text does not contain two valid matrices.')
    except ValueError:
        raise ValueError('Input text does not contain two valid matrices.')
    except matrix.InvalidMatrixReprError:
        raise ValueError('Input text does not contain two valid matrices.')

    if m1.getNumRows() != m2.getNumRows() or m1.getNumCols() != m2.getNumCols():
        raise ValueError('Input text contains two matrices with different ' +\
            'number of rows or columns.')

    return (m1, m2)


def printHelp(stream):
    """Prints program help to the selected stream.

    stream - stream into which the help will be printed
    """
    helpText =\
"""Program for computing mixed Nash equilibrium (MNE) in 2-player games using the Lemke-Howson algorithm.

Usage: python lh.py < inputgame.txt

Program expects two matrices with payoffs on the standard input in the following format:
    a11 a12 ... a1N\\n
    ...
    aM1 aM2 ... aMN\\n
    \\n
    b11 b12 ... b1N\\n
    ...
    bM1 bM2 ... bMN\\n

aXY are payoffs for the first player and bXY are payoffs for the second player.

Restrictions:
 - the game must be nondegenerative
"""
    stream.write(helpText)


def printGameInfo(m1, m2, eq, stream):
    """Prints game information to the selected stream.

    m1 - matrix of the first player (Matrix)
    m2 - matrix of the second player (Matrix)
    eq - game equilibrium (tuple containing two tuples)
    stream - stream into which the game info will be printed
    """
    stream.write('Player 1:\n')
    stream.write(repr(m1))
    stream.write('\n')
    stream.write('Player 2:\n')
    stream.write(repr(m2))
    stream.write('\n')
    stream.write('Found MNE: ')
    printEquilibrium(eq, stream)
    stream.write('\n')


def printEquilibrium(eq, stream):
    """Prints the selected equilibrium to the selected stream
    using the repr(eq) function. If some part of the equilibrium
    contains only one strategy, it will omit the redundant comma
    from the output (e.g., (1/2,) will be replaced with (1/2)).
    0/1 is replaced with 0 and 1/1 is replaced with 1.

    eq - equilibrium to be printed (tuple containing two tuples)
    stream - stream into which the equilibrium will be printed
    """
    eqToPrint = repr(eq)

    # Perform some cosmetical enhancements:
    # 1) Removed the redundant comma if a part of the equilibrium
    # is a single strategy
    eqToPrint = eqToPrint.replace(',)', ')')
    # 2) Transform 0/1 into 0
    eqToPrint = eqToPrint.replace('0/1', '0')
    # 3) Transform 1/1 into 1
    eqToPrint = eqToPrint.replace('1/1', '1')

    # stream.write(eqToPrint)

    return eqToPrint
