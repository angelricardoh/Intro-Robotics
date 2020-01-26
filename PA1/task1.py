def mysum1(A):
    result = 0
    for integer in A:
        result += integer
    return result


def myfib1(n):
    if n <= 1:
        return n
    return myfib1(n - 1) + myfib1(n - 2)


print(mysum1([1, 5, 7]))
print(mysum1([]))
print(mysum1([-5, 3]))
print(myfib1(3))
print(myfib1(5))
print(myfib1(7))
