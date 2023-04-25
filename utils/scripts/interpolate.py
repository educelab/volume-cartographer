import numpy as np
from pathlib import Path
import csv

#read in keypoints from csv
def getKeypoints(polyLinePointsCSV):
    keyPoints = []
    with open(polyLinePointsCSV, newline='') as csvfile:
        polyRead = csv.reader(csvfile)
        for row in polyRead:
            kp = np.array(row, dtype=np.float32).reshape((-1, 3))
            keyPoints.append(kp)

    return keyPoints

def write_ordered_vcps(output_path, pointset):
    # Open output file and write ASCII header
    with Path(output_path).open('wt') as file:
        file.writelines([
            f'width: {pointset.shape[1]}\n',
            f'height: {pointset.shape[0]}\n',
            f'dim: {pointset.shape[2]}\n',
            'ordered: true\n',
            'type: double\n',
            'version: 1\n',
            '<>\n'
        ])

    # Reopen in binary append mode
    with Path(output_path).open('ab') as file:
        # Write as doubles
        pointset.astype(np.float64).tofile(file)


def interpolate_points(keyPoints):
    #XY interpolation
    keyPointsNew = []
    #print(keyPoints[1])
    sliceNum = 0
    xyNumPoints = 5
    for row in keyPoints:
        newRow = []
        for index, point in enumerate(row):
            if index == row.shape[0] - 1:
                newRow.append(point)
                break
            nextPt = row[index + 1]
            deltaXY = (nextPt - point)/xyNumPoints

            for t in range(xyNumPoints):
                newXY = point + (t*deltaXY)
                newRow.append(newXY)
        keyPointsNew.append(np.array(newRow))

    keyPoints = keyPointsNew
    # New Z interpolation #
    # For each row
    cloud = []
    for row_idx, row in enumerate(keyPoints):
        # Add the last row of points and finish
        if row_idx == len(keyPoints) - 1:
            new_row = []
            for point in row:
                new_row.append(point)
            cloud.append(new_row)
            break

        # Get the next row
        next_row = keyPoints[row_idx + 1]

        # Iterate over each new interpolated row
        num = int((next_row[0, 2] - row[0, 2]).item())
        for t in range(num):

            # Iterate over each point in the new row
            new_row = []
            for pt_idx, point in enumerate(row):
                next_point = next_row[pt_idx]
                delta = (next_point - point) / num
                new_point = point + t * delta
                new_row.append(new_point)
            # Add the new row to the output cloud
            cloud.append(new_row)
    # Returns ordered pointset of shape (rows, width, 3)
    print(cloud)
    return np.array(cloud)


def main():
    # Test keypoints
    # Replace with your existing key point loading function
    # keyPoints = [
    #     np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=np.float32),
    #     np.array([[0, 0, 5], [1, 0, 5], [2, 0, 5]], dtype=np.float32)
    # ]
    keyPoints = getKeypoints('fullWrapLayerOne.csv')

    # Interpolate keypoints
    cloud = interpolate_points(keyPoints)

    # Save cloud ordered vcps
    write_ordered_vcps("fullWrapLayerOne.vcps", cloud)


if __name__ == '__main__':
    main()
