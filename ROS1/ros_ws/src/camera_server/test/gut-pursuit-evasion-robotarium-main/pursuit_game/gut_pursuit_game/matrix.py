# vim:fileencoding=utf8
#
# Project: Implementation of the Lemke-Howson algorithm for finding MNE
# Author:  Petr Zemek <s3rvac@gmail.com>, 2009
#

"""This module provides a representation of matrices and some common
functions operating on matrices."""


import re


class InvalidMatrixReprError(Exception):
	"""Exception to be raised when an invalid matrix is encountered."""
	pass


def fromText(text, itemFromStrFunc=int):
    """Returns a matrix created from the selected text (string).

    string - string from which the matrix will be created
    itemFromStrFunc - unary function, which will be called on each parsed item
                      (string) before storing it into the matrix

    The input text must be in the same format as described in the
    Matrix.__repr__() function description. The last new line can be ommited.
    Redundant white space will be removed.

    Raises InvalidMatrixReprError if the text cannot be converted into a matrix.
    """
    if text == '':
        raise InvalidMatrixReprError('The input text is empty.')

    # Make sure that the text ends with a new line
    if text[-1] != '\n':
        text = text + '\n'

    lines = text.split('\n')
    # Remove the last line that contain only the new line
    lines = lines[:-1]

    # RegExp for a matrix item
    matrixItemRE = re.compile(r'\s*(\S+)\s*', re.U)

    # Get the number of rows and columns
    rows = len(lines)
    cols = len(re.findall(matrixItemRE, lines[0]))

    # Parse the input text into a matrix
    try:
        m = Matrix(rows, cols)
        for i in range(0, rows):
            rowItems = re.findall(matrixItemRE, lines[i])
            for j in range(0, cols):
                m.setItem(i + 1, j + 1, itemFromStrFunc(rowItems[j]))
        return m
    except IndexError as e:
        raise InvalidMatrixReprError(e.message)


class Matrix(object):
    """This class represents a matrix in a two dimensional space.

    Objects of this class are mutable."""

    def __init__(self, rows, cols):
        """Creates a matrix with the selected number of rows and columns.

        rows - number of rows
        cols - numer of columns

        All elements are initialized to zero.

        Preconditions:
            - rows > 0
            - cols > 0

        Raises ValueError if some of the preconditions are not met.
        """
        if rows <= 0:
            raise ValueError('Number of matrix rows must be greater than zero.')
        if cols <= 0:
            raise ValueError('Number of matrix cols must be greater than zero.')

        self.__rows = rows
        self.__cols = cols

        # Create the matrix and initialize all elements to zero
        self.__m = []
        for i in range(1, self.__rows + 1):
            row = []
            for j in range(1, self.__cols + 1):
                row.append(0)
            self.__m.append(row)

    def getNumRows(self):
        """Returns the number of rows of the matrix."""
        return self.__rows

    def getNumCols(self):
        """Returns the number of rows of the matrix."""
        return self.__cols

    def setItem(self, i, j, val):
        """Sets a new value of the item on the ith row and jth column.

        i - row number
        j - columns number
        val - value to be set

        Preconditions:
            - 0 < i < getNumRows()
            - 0 < j < getNumCols()

        Raises IndexError if some of the preconditions are not met.
        """
        if i < 0:
            raise IndexError('Row index must be nonnegative.')
        if j < 0:
            raise IndexError('Column index must be nonnegative.')

        self.__m[i - 1][j - 1] = val

    def getItem(self, i, j):
        """Returns the item on the ith row and jth column.

        i - row number
        j - columns number

        Preconditions:
            - 0 < i < getNumRows()
            - 0 < j < getNumCols()

        Raises IndexError if some of the preconditions are not met.
        """
        if i < 0:
            raise IndexError('Row index must be nonnegative.')
        if j < 0:
            raise IndexError('Column index must be nonnegative.')

        return self.__m[i - 1][j - 1]

    def __repr__(self):
        """Returns a printable representation of the matrix (string).

        The result will be a string in the following form:
        a11 a12 a13 ... a1N\n
        ...
        aN1 aN2 aN3 ... aMN\n

        where aXY is the string representation of the item on the Xth row
        and Yth column. M is the number of matrix rows and N is the number
        of matrix columns.
        """
        res = ''
        for i in range(1, self.getNumRows() + 1):
            for j in range(1, self.getNumCols() + 1):
                res += repr(self.getItem(i, j))
                res += ' ' if j < self.getNumCols() else '\n'
        return res

    def __eq__(self, other):
        """Returns true if this matrix is equal to the other matrix.
        Two matrices are considered equal if they have the same number of rows,
        columns and every item on indices i,j in the first matrix is equal to
        the item on the same indices in the second matrix.
        """
        # Check the number of rows and columns
        if self.getNumRows() != other.getNumRows() or\
                self.getNumCols() != other.getNumCols():
            return False

        # Check items
        for i in range(1, self.getNumRows() + 1):
            for j in range(1, self.getNumCols() + 1):
                if self.getItem(i, j) != other.getItem(i, j):
                    return False

        return True

    def __ne__(self, other):
        """Returns True, if this matrix is NOT equal to the other matrix, False
        otherwise. See __eq__() for more details about equality. """
        return not (self == other)
