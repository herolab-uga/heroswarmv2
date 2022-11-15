# vim:fileencoding=utf8
#
# Project: Implementation of the Lemke-Howson algorithm for finding MNE
# Author:  Petr Zemek <s3rvac@gmail.com>, 2009
#

"""This module contains a Lemke-Howson algorithm implementation
and various functions that are used in that algorithm.
"""


import matrix
import rational
from functools import reduce


def normalizeMatrices(m1, m2):
    """Returns normalized selected matrices in a tuple.
    Normalized matrix does not have any row with all zeros, nor
    any column with all zeros. Also any element will contain positive
    number.

    m1 - first matrix to be normalized (Matrix)
    m2 - second matrix to be normalized (Matrix)

    The normalization is done by adding a proper constant to all
    item of both matrices (the least possible constant + 1 is chosen).
    If both matrices do not have any negative items, nor any items
    equal to zero, no constant is added.
    """
    ms = (m1, m2)

    # Check for the least value in both matrices
    lowestVal = m1.getItem(1, 1)
    for m in ms:
        for i in range(1, m.getNumRows() + 1):
            for j in range(1, m.getNumCols() + 1):
                if m.getItem(i, j) < lowestVal:
                    lowestVal = m.getItem(i, j)

    normMs = (matrix.Matrix(m1.getNumRows(), m1.getNumCols()),
               matrix.Matrix(m2.getNumRows(), m2.getNumCols()))

    # Copy all items from both matrices and add a proper constant
    # to all values
    cnst = 0 if lowestVal > 0 else abs(lowestVal) + 1
    for k in range(0, len(normMs)):
        for i in range(1, ms[k].getNumRows() + 1):
            for j in range(1, ms[k].getNumCols() + 1):
                normMs[k].setItem(i, j, ms[k].getItem(i, j) + cnst)

    return normMs


def createTableaux(m1, m2):
    """Creates a tableaux from the two selected matrices.

    m1 - first matrix (Matrix instance)
    m2 - second matrix (Matrix instance)

    Preconditions:
        - m1 must have the same number of rows and columns as m2

    Raises ValueError if some of the preconditions are not met.
    """
    if m1.getNumRows() != m2.getNumRows() or m1.getNumCols() != m2.getNumCols():
        raise ValueError('Selected matrices does not have the same number ' +\
                'of rows and columns')

    # The total number of strategies of both players
    S = m1.getNumRows() + m1.getNumCols()

    # The tableaux will have S rows, because there are S slack variables
    # and S + 2 columns, because the first column is the index of the basis
    # in the current column and the second column is initially all 1s
    t = matrix.Matrix(S, S + 2)

    # Initialize the first column (index of the currect basis variable).
    # Because there are only slack variables at the beginning, initialize
    # it to a sequence of negative numbers starting from -1.
    for i in range(1, t.getNumRows() + 1):
        t.setItem(i, 1, -i)

    # Initialize the second column to all 1s (current value of all basis)
    for i in range(1, t.getNumRows() + 1):
        t.setItem(i, 2, 1)

    # Initialize indices from the first matrix
    for i in range(1, m1.getNumRows() + 1):
        for j in range(1, m1.getNumCols() + 1):
            t.setItem(i, m1.getNumRows() + j + 2, -m1.getItem(i, j))

    # Initialize indices from the second matrix
    for i in range(1, m2.getNumRows() + 1):
        for j in range(1, m2.getNumCols() + 1):
            t.setItem(m1.getNumRows() + j, i + 2, -m2.getItem(i, j))

    return t


def makePivotingStep(t, p1SCount, ebVar):
    """Makes a single pivoting step in the selected tableaux by
    bringing the selected variable into the basis. All changes are done
    in the original tableaux. Returns the variable that left the basis.

    t - tableaux (Matrix)
    p1SCount - number of strategies of player 1 (number)
    ebVar - variable that will enter the basis (number)

    Preconditions:
        - 0 < abs(ebVar) <= t.getNumRows()
        - 0 < p1SCount < t.getNumRows()

    Raises ValueError if some of the preconditions are not met.
    """
    # 1st precondition
    if abs(ebVar) <= 0 or abs(ebVar) > t.getNumRows():
        raise ValueError('Selected variable index is invalid.')
    # 2nd precondition
    if p1SCount < 0 or t.getNumRows() <= p1SCount:
        raise ValueError('Invalid number of strategies of player 1.')

    # Returns the column corresponding to the selected variable
    def varToCol(var):
        # Apart from players matrices values, there are 2 additional
        # columns in the tableaux
        return 2 + abs(var)

    # Returns the list of row numbers which corresponds
    # to the selected variable
    def getRowNums(var):
        # Example (for a game 3x3):
        #   -1,-2,-3,4,5,6 corresponds to the first part of the tableaux
        #   1,2,3,-4,-5,-6 corresponds to the second part of the tableaux
        if -p1SCount <= var < 0 or var > p1SCount:
            return list(range(1, p1SCount + 1))
        else:
            return list(range(p1SCount + 1, t.getNumRows() + 1))

    # Check which variable should leave the basis using the min-ratio rule
    # (it will have the lowest ratio)
    lbVar = None
    minRatio = None
    # Check only rows in the appropriate part of the tableaux
    for i in getRowNums(ebVar):
        if t.getItem(i, varToCol(ebVar)) < 0:
            ratio = -rational.Rational(t.getItem(i, 2)) /\
                    t.getItem(i, varToCol(ebVar))
            if minRatio == None or ratio < minRatio:
                minRatio = ratio
                lbVar = t.getItem(i, 1)
                lbVarRow = i
                lbVarCoeff = t.getItem(i, varToCol(ebVar))

    # Update the row in which the variable that will leave the basis was
    # found in the previous step
    t.setItem(lbVarRow, 1, ebVar)
    t.setItem(lbVarRow, varToCol(ebVar), 0)
    t.setItem(lbVarRow, varToCol(lbVar), -1)
    for j in range(2, t.getNumCols() + 1):
        newVal = rational.Rational(t.getItem(lbVarRow, j)) / abs(lbVarCoeff)
        t.setItem(lbVarRow, j, newVal)

    # Update other rows in the appropriate part of the tableaux
    for i in getRowNums(ebVar):
        if t.getItem(i, varToCol(ebVar)) != 0:
            for j in range(2, t.getNumCols() + 1):
                newVal = t.getItem(i, j) + t.getItem(i, varToCol(ebVar)) *\
                        t.getItem(lbVarRow, j)
                t.setItem(i, j, newVal)
            t.setItem(i, varToCol(ebVar), 0)

    return lbVar


def getEquilibrium(t, p1SCount):
    """Returns the equilibrium from the given tableaux. The returned result
    might contain mixed strategies like (1/3, 0/1), so normalization is need to
    be performed on the result.

    t - tableaux (Matrix)
    p1SCount - number of strategies of player 1 (number)

    Preconditions:
        - 0 < p1SCount < t.getNumRows()
        - first column of the matrix must contain each number from 1 to
          t.getNumRows (inclusive, in absolute value)

    Raises ValueError if some of the preconditions are not met.
    """
    # 1st precondition
    if p1SCount < 0 or t.getNumRows() <= p1SCount:
        raise ValueError('Invalid number of strategies of player 1.')
    # 2nd precondition
    firstColNums = []
    for i in range(1, t.getNumRows() + 1):
        firstColNums.append(abs(t.getItem(i, 1)))
    for i in range(1, t.getNumRows() + 1):
        if not i in firstColNums:
            raise ValueError('Invalid indices in the first column of the tableaux.')

    # I decided to use a list instead of a tuple, because I need
    # to modify it (tuples are immutable)
    eqs = t.getNumRows() * [0]

    # Equilibrium is in the second column of the tableaux
    for i in range(1, t.getNumRows() + 1):
        # Strategy
        strat = t.getItem(i, 1)
        # Strategy probability
        prob = t.getItem(i, 2)
        # If the strategy index or the probability is lower than zero,
        # set it to zero instead
        eqs[abs(strat) - 1] = rational.Rational(0) if (strat < 0 or prob < 0) else prob

    # Convert the found equilibrium into a tuple
    return (tuple(eqs[0:p1SCount]), tuple(eqs[p1SCount:]))


def normalizeEquilibrium(eq):
    """Normalizes and returns the selected equilibrium (every probability
    in a players mixed strategy will have the same denominator).

    eq - equilibrium to be normalized (tuple of two tuples of Rationals)

    Preconditions:
        - len(eq) == 2 and len(eq[0] > 0) and len(eq[1]) > 0
        - eq[x] must contain a non-empty tuple of Rationals for x in {1,2}

    Raises ValueError if some of the preconditions are not met.
    """
    # 1st precondition
    if len(eq) != 2 or (len(eq[0]) == 0 or len(eq[1]) == 0):
        raise ValueError('Selected equilibrium is not valid.')
    # 2nd precondition
    for i in range(0, 2):
        for j in range(0, len(eq[i])):
            if not isinstance(eq[i][j], rational.Rational):
                raise ValueError('Selected equilibrium contains a ' +\
                    'non-rational number.')

    # Normalizes a single part of the equilibrium (the normalization
    # procedure is the same as with vectors)
    def normalizeEqPart(eqPart):
        probSum = reduce(lambda x, y: x + y, eqPart, 0)
        return tuple([x * probSum.recip() for x in eqPart])

    return (normalizeEqPart(eq[0]), normalizeEqPart(eq[1]))


def lemkeHowson(m1, m2):
    """Runs the Lemke-Howson algorithm on the selected two matrices and
    returns the found equilibrium in mixed strategies. The equilibrium
    will be normalized before it is returned.

    m1 - matrix of profits of the first player (Matrix)
    m2 - matrix of profits of the second player (Matrix)

    Preconditions:
        - m1 must have the same number of rows and columns as m2
        - the game specified by m1 and m2 must be nondegenerative

    Raises ValueError if the first precondition is not met.
    """
    # Before we start, we need to normalize both matrices
    # to ensure some assumptions about values in both matrices
    (normM1, normM2) = normalizeMatrices(m1, m2)

    # Create the tableaux that will be used in the pivoting procedure
    t = createTableaux(normM1, normM2)

    # Make pivoting steps until the equilibrium is found
    # (the variable that left the basis is the same (in absolute value)
    # as the variable that we used as an initial pivot)
    p1SCount = normM1.getNumRows()
    initBasisVar = 1
    leftBasisVar = makePivotingStep(t, p1SCount, initBasisVar)
    while abs(leftBasisVar) != initBasisVar:
        leftBasisVar = makePivotingStep(t, p1SCount, -leftBasisVar)

    # Get the equilibrium from the resulting tableaux,
    # normalize it and return it
    return normalizeEquilibrium(getEquilibrium(t, p1SCount))
