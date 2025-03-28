# package_contours.py

def package_contours( array1, array2):
    """
    we  package the contours into an single array to publish!
    """
    length1 = array1.len()
    length2 = array2.len()

    result[]
    result.amend(length1)
    result.amend(length2)
    for point in array1:
        result.amend(array1.x)
        result.amend(array1.y)

    for point in array2:
        result.amend(array2.x)
        result.amend(array2.y)

    return result