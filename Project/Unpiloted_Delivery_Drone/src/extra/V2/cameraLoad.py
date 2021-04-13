"""
    Load Calibrated Files.
"""



def getCameraMat():
    """
    - Get the camera matrix from config folder.

    returns the camera matrix.
    """
    camera_matrix_store = []

    _fileCam = open("src/config/camera_matrix.npy", "r")
    camSplit = _fileCam.read().split()

    # Convert Str -> Float
    for i in range(0, len(camSplit)):
        camera_matrix_store.append(float(camSplit[i]))

    _fileCam.close()

    return camera_matrix_store


def getDistortionMat():
    """
    - Get the distortion matrix from config folder.

    returns the distortion matrix.
    """
    distortion_coeff_store = []

    _fileDis = open("src/config/distortion_coeff.npy", "r")
    disSplit = _fileDis.read().split()

    for i in range(0, len(disSplit)):
        distortion_coeff_store.append(float(disSplit[i]))

    _fileDis.close()


    return distortion_coeff_store

