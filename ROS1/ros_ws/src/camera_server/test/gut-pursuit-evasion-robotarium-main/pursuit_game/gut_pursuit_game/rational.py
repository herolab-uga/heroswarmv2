# vim:fileencoding=utf8
#
# Project: Implementation of the Lemke-Howson algorithm for finding MNE
# Author:  Petr Zemek <s3rvac@gmail.com>, 2009
#

"""This module provides a class Rational as a representation
of rational numbers with common operations over them.
"""


import re


class InvalidRationalReprError(Exception):
	"""Exception to be raised when an invalid rational number is encountered."""
	pass


def fromText(text):
    """Creates and returns a Rational class instance from the selected
    text (string).

    text - textual representation of a rational number (string)

    text can contain redundant whitespaces.

    The following forms are accepted:
        x
        x/y
        -x/y
    where x and y are positive numbers.

    Raises InvalidRationalReprError if the text cannot be converted into
    a rational number.
    """
    rationalRegExp = re.compile(r'^\s*(-?\d+)\s*(/\s*(\d+))?\s*$', re.U)
    m = re.match(rationalRegExp, text)
    if m != None:
        a = int(m.group(1))
        b = int(m.group(3)) if m.group(2) != None else 1
        return Rational(a, b)
    else:
        raise InvalidRationalReprError


def _gcd(a, b):
    """Returns the greatest common divisor of a and b."""
    if a < b:
        a, b = b, a
    while b != 0:
        a, b = b, a % b
    return a


class Rational:
    """This class represents rational numbers (nominator/denominator).

    Instances of this class are immutable.
    """

    def __init__(self, a, b=1):
        """Creates a new rational number.

        If a and b are numbers (int, long), then the resulting rational number
        will be in the form a/b. If just a or just b is negative, then the
        rational number will be negative. If both (a and b) are negative, then
        the rational number will be positive. If a and b are commensurable,
        then they will be divided by their greatest common divisor (e.g. 5/10
        will be transformed into 1/2).

        If just a is given and it's a rational number, then the resulting
        rational number will be equal to a.

        Preconditions:
            - b must be nonzero
            - a and b must be immutable numeric objects
            - if a is a rational number, b must be 1

        Raises ValueError if some of the preconditons are not met.
        """
        if b == 0:
            raise ValueError('b must be nonzero.')

        try:
            # Lets suppose that a is a rational number
            self.__a = a.nom()
            self.__b = a.denom()

            if b != 1:
                raise ValueError('If a is a rational number, b must be 1.')
        except AttributeError:
            # a and b must be ordinary numbers
            self.__a = a
            self.__b = b

            # Sign normalization
            if self.__b < 0:
                self.__a = -self.__a
                self.__b = abs(self.__b)

            # Commensurability normalization
            d = _gcd(abs(self.__a), abs(self.__b))
            self.__a //= d
            self.__b //= d

    def nom(self):
        """Returns the nominator part of the number (if the rational number
        was negative, the returned result is also negative).
        """
        return self.__a

    def denom(self):
        """Returns the denominator part of the number (if the rational number
        was negative, the returned result is positive).
        """
        return self.__b

    def recip(self):
        """Returns the reciprocal version of this rational (i.e. nominator
        will be switched with denominator).
        """
        return Rational(self.denom(), self.nom())

    def __add__(self, r):
        """Adds r to self (r can be a number or other Rational).

        Returned number is normalized (based on Commensurability).
        """
        try:
            # Adding other rational
            tmpa = self.nom() * r.denom() + r.nom() * self.denom()
            tmpb = self.denom() * r.denom()
        except AttributeError:
            # Adding a number
            tmpa = self.nom() + self.denom() * r
            tmpb = self.denom()
        return Rational(tmpa, tmpb)

    def __radd__(self, r):
        """Does the same as __add__()."""
        return self.__add__(r)

    def __mul__(self, r):
        """Multiplies r to self (r can be a number or other Rational).

        Returned number is normalized (based on Commensurability).
        """
        try:
            # Multing with other rational
            tmpa = self.nom() * r.nom()
            tmpb = self.denom() * r.denom()
        except AttributeError:
            # Multing with a number
            tmpa = self.nom() * r
            tmpb = self.denom()
        return Rational(tmpa, tmpb)

    def __rmul__(self, r):
        """Does the same as __mul__()."""
        return self.__mul__(r)

    def __truediv__(self, r):
        """Divides self with r (r can be a number or other Rational).

        Returned number is normalized (based on Commensurability).
        """
        try:
            # Dividing with other rational
            tmpa = self.nom() * r.denom()
            tmpb = self.denom() * r.nom()
        except AttributeError:
            # Dividing with a number
            tmpa = self.nom()
            tmpb = self.denom() * r
        return Rational(tmpa, tmpb)

    def __rdiv__(self, r):
        """Does the same as __div__()."""
        return self.__div__(r)

    def __abs__(self):
        """Returns the absolute value of this rational."""
        return Rational(abs(self.nom()), self.denom())

    def __neg__(self):
        """Returns the negated value if this rational."""
        return Rational(-self.nom(), self.denom())

    def __eq__(self, r):
        """Returns True, if this object is equal to the r object (rational
        number or an ordinary number), False otherwise.
        """
        try:
            # Compare with other rational
            return self.nom() == r.nom() and self.denom() == r.denom()
        except AttributeError:
            # Compare with a number
            return self.nom() == r and self.denom() == 1

    def __ne__(self, r):
        """Returns True, if this object is NOT equal to the r object (rational
        number or an ordinary number), False otherwise.
        """
        return not (self == r)

    def __lt__(self, r):
        """Returns True if self < r, False otherwise. r can be a rational
        number of an ordinary number.
        """
        try:
            # Compare with other rational
            # Transform both rationals to the same denominator and compare
            # their nominators
            return (self.nom() * r.denom()) < (r.nom() * self.denom())
        except AttributeError:
            # Compare with a number
            return self.nom() < (r * self.denom())

    def __le__(self, r):
        """Returns True if self <= r, False otherwise. r can be a rational
        number of an ordinary number.
        """
        return self < r or self == r

    def __gt__(self, r):
        """Returns True if self > r, False otherwise. r can be a rational
        number of an ordinary number.
        """
        return not (self < r) and self != r

    def __ge__(self, r):
        """Returns True if self >= r, False otherwise. r can be a rational
        number of an ordinary number.
        """
        return self > r or self == r

    def __str__(self):
        """Returns the string representation of the current rational number
        in the form a/b (if the number is negative, then there will be a '-'
        character before a).
        """
        return "%d/%d" % (self.nom(), self.denom())

    def __repr__(self):
        """Returns the same as __str__()."""
        return self.__str__()
