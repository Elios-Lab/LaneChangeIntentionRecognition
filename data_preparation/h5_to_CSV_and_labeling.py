# H5 to CSV and labeling script
# This code take as input h5 files in the folder h5 and it selects all the important signals for lane change intention available in real-world scenarios. 
# Then the position of the car is analyzed to find the starting timestamps of the lane change maneuver (center of the car change the lane).
# All the data before a lane change are labelled from 0-4 seconds lane change, 4.1 as free-ride, and "-" to ignore 1.5 seconds after a lane change.
# The output is a csv file in the dataset folder. 

#Change here name of the files (e.g. log_001 for the file h5/log_001.h5)
h5_array = ["log_001"]
plotGraphs = False

# Don't change the following lines, they are used to produce the csv file in dataset folder
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import utility.h5_functions as h5

for h5_name in h5_array:
    #Read the h5 file
    file_h5="./h5/"+h5_name+".h5"
    
    h5_dict = h5.read_h5_content(file_h5)

    #Important data of the ego vehicle sheet
    fileTime = h5_dict["egoVehicle"]["FileTime"]
    latAcceleration = h5_dict["egoVehicle"]["LatAcceleration"]
    longAcceleration = h5_dict["egoVehicle"]["LongAcceleration"]
    brakePedalPos = h5_dict["egoVehicle"]["BrakePedalPos"]
    directionIndicator = h5_dict["egoVehicle"]["DirectionIndicator"]
    steeringAngle = h5_dict["egoVehicle"]["SteeringAngle"]
    throttlePedalPos = h5_dict["egoVehicle"]["ThrottlePedalPos"]
    vehicleSpeed = h5_dict["egoVehicle"]["VehicleSpeed"]
    yawRate = h5_dict["egoVehicle"]["YawRate"]

    #one hot encoding of the direction indicator: 1 is left and 2 is right
    leftDirectionIndicator = [1 if x == 1 else 0 for x in directionIndicator]
    rightDirectionIndicator = [1 if x == 2 else 0 for x in directionIndicator]

    #Important data of the laneLines sheet
    egoLaneWidth = h5_dict["laneLines"]["EgoLaneWidth"]
    sLaneLine = h5_dict["laneLines"]["sLaneLine"]
    dy=sLaneLine["Dy"]
    yawAngle=sLaneLine["YawAngle"]
    curvature=sLaneLine["Curvature"]
    curvatureDx=sLaneLine["CurvatureDx"]
    rightLaneLine = []
    leftLaneLine = []
    yawAngleRightLine = []
    yawAngleLeftLine = []
    curvatureRightLine = []
    curvatureLeftLine = []
    curvatureDxRightLine = []
    curvatureDxLeftLine = []

    for d in dy:
        rightLaneLine.append(d[0])
        leftLaneLine.append(d[1])
        
    for d in yawAngle:
        yawAngleRightLine.append(d[0])
        yawAngleLeftLine.append(d[1])

    for d in curvature:
        curvatureRightLine.append(d[0])
        curvatureLeftLine.append(d[1])
        
    for d in curvatureDx:
        curvatureDxRightLine.append(d[0])
        curvatureDxLeftLine.append(d[1])

    print("The width of the ego lane is: "+str(egoLaneWidth[0]))

    h5_dict["objects"].keys()
    print("Number of objects: "+str(h5_dict["objects"]["NumberOfObjects"][0]))

    #important data of the positioning sheet
    lanes = [0]

    currentLane = 0
    previousRightLaneLine = rightLaneLine[0]
    for i in range(len(rightLaneLine)):
        #change lane if the distance between the right lane line and the previous right lane line is greater than the half of the ego lane width
        if abs(rightLaneLine[i] - previousRightLaneLine) > egoLaneWidth[i]/2:
            #if current distance from right line is less than half of the ego lane width, the car is in the left lane respect the previous
            if rightLaneLine[i] < egoLaneWidth[i]/2:
                currentLane+=1
                if currentLane not in lanes:
                    lanes.append(currentLane)
            else:
                currentLane-=1
                if currentLane not in lanes:
                    lanes.append(currentLane)
        previousRightLaneLine = rightLaneLine[i]


    print("The car travelled in "+str(len(lanes))+" lanes")
    print("The starting lane is: "+str(lanes[0]))
    print("The lanes are: "+str(lanes))
    #0 is the starting lane, negative are the left lanes and positive are the rigth lanes

    #startLane automatically detected
    startLane = abs(min(lanes)) # 1 = left, 0 = right
    print(startLane)

    #if rightLaneline change from previous value in asbolute value is greater than 3/4 of the road, it means that the car has changed lane
    currentLane = []
    previousRightLaneLine = rightLaneLine[0]
    for i in range(len(rightLaneLine)):
        if abs(rightLaneLine[i] - previousRightLaneLine) > egoLaneWidth[i]/2:
            if startLane == 1:
                startLane = 0
            else:
                startLane = 1
        currentLane.append(startLane)
        previousRightLaneLine = rightLaneLine[i]      
   
    samples_in_one_second = 10
    def plotRoad(leftOvertakeIndex=None,rightOvertakeIndex=None, startIndex=None, endIndex=None, splitSeconds=False):
        value=[]
        for i in range(len(currentLane)):
            #to have in the upper part of the plot the most external lane on the left and in the lower part the most external lane on the right 
            value.append((currentLane[i] * egoLaneWidth[i] + rightLaneLine[i]))
        if(startIndex is None or endIndex is None):
            plt.plot(fileTime,value)
        else:
            plt.plot(fileTime[startIndex:endIndex],value[startIndex:endIndex])
        plt.title('Position of the vehicle in the road')
        plt.xlabel('Seconds')
        plt.ylabel('Road Lanes (meters)')

        # Adding horizontal lines at intervals of 3.75 units
        for i in range(3):
            plt.axhline(y=egoLaneWidth[0] * (i), color='k', linestyle='-')

        # Adding vertical lines for overtaking
        if leftOvertakeIndex is not None:
            for i in leftOvertakeIndex:
                plt.axvline(x=fileTime[i], color='r', linestyle='-')
            
        if rightOvertakeIndex is not None:
            for i in rightOvertakeIndex:
                plt.axvline(x=fileTime[i], color='b', linestyle='-')
                
        if splitSeconds and ((leftOvertakeIndex is not None and len(leftOvertakeIndex)==1) or (rightOvertakeIndex is not None and len(rightOvertakeIndex)==1)):
            overtakingTimestamp = (leftOvertakeIndex if (leftOvertakeIndex is not None and len(leftOvertakeIndex)==1) else rightOvertakeIndex)[0]        
            # Adding a colored band for the lane change
            plt.axvspan(fileTime[overtakingTimestamp - int(samples_in_one_second)], fileTime[overtakingTimestamp], color='r', alpha=0.3)

            # Adding a colored band for 1 second before
            plt.axvspan(fileTime[overtakingTimestamp - 2*samples_in_one_second], fileTime[overtakingTimestamp - (samples_in_one_second)], color='g', alpha=0.3)

            #adding a colored band for 2 second before
            plt.axvspan(fileTime[overtakingTimestamp - 3 * samples_in_one_second], fileTime[overtakingTimestamp - (2 * samples_in_one_second)], color='b', alpha=0.3)

            #adding a colored band for 3 second before
            plt.axvspan(fileTime[overtakingTimestamp - 4 * samples_in_one_second], fileTime[overtakingTimestamp - (3 * samples_in_one_second)], color='y', alpha=0.3)
            
            # Creating patches for the legend
            lane_change_patch = mpatches.Patch(color='r', alpha=0.3, label='0 second to lane Change')
            one_sec_before_patch = mpatches.Patch(color='g', alpha=0.3, label='1 second to lane Change')
            two_sec_before_patch = mpatches.Patch(color='b', alpha=0.3, label='2 seconds to lane Change')
            three_sec_before_patch = mpatches.Patch(color='y', alpha=0.3, label='3 seconds to lane Change')

            # Adding the legend
            plt.legend(handles=[lane_change_patch, one_sec_before_patch, two_sec_before_patch, three_sec_before_patch], loc='best')

        plt.show()
        
    if plotGraphs:
        plotRoad()
            
    #plot currentLane
    if plotGraphs:
        plt.plot(currentLane)
        plt.title('Current Lane')
        plt.xlabel
        plt.show()
        
    #if rightLaneline change from previous value in asbolute value is greater than 3/4 of the road, it means that the car has changed lane
    count=0
    indexesOvertake = []
    rightOvertakeIndexes = []
    leftOvertakeIndexes = []
    previousRightLaneLine = rightLaneLine[0]
    for i in range(len(rightLaneLine)):
        if abs(rightLaneLine[i] - previousRightLaneLine) > egoLaneWidth[i]*2/4:
            print("now:"+str(rightLaneLine[i])+" , previous:"+str(previousRightLaneLine))
            if startLane == 1:
                startLane = 0
            else:
                startLane = 1
            count+=1
            indexesOvertake.append(i)
            if rightLaneLine[i] < egoLaneWidth[i]/2:
                leftOvertakeIndexes.append(i)
            else:
                rightOvertakeIndexes.append(i)
        previousRightLaneLine = rightLaneLine[i]
    print("total lane changes: " +str(count))
    print("indexes of lane changes: "+str(indexesOvertake))    

    if plotGraphs:
        plotRoad(leftOvertakeIndexes, rightOvertakeIndexes)

    #Zoom in the left overtake
    if plotGraphs:
        for leftOvertake in leftOvertakeIndexes:
            plotRoad([leftOvertake], None, leftOvertake-20, leftOvertake+5)
            
    #Zoom in the right overtake
    if plotGraphs:
        for rightOvertake in rightOvertakeIndexes:
            plotRoad(None, [rightOvertake], rightOvertake-20, rightOvertake+5)       
            
    #Zoom in the left overtake
    if plotGraphs:
        for leftOvertake in leftOvertakeIndexes:
            plotRoad([leftOvertake], None, leftOvertake-101, leftOvertake+2,True)
            break        
            
    #Zoom in the left overtake
    if plotGraphs:
        for leftOvertake in leftOvertakeIndexes:
            plotRoad([leftOvertake], None, leftOvertake-41, leftOvertake+2,True)
            
    #Zoom in the right overtake
    if plotGraphs:
        for rightOvertake in rightOvertakeIndexes:
            plotRoad(None, [rightOvertake], rightOvertake-41, rightOvertake+2,True)       
                        
    LatPosition = h5_dict["objects"]["sObject"]["LatPosition"]
    LatVelocity = h5_dict["objects"]["sObject"]["LatVelocity"]
    LongPosition = h5_dict["objects"]["sObject"]["LongPosition"]
    LongVelocity = h5_dict["objects"]["sObject"]["LongVelocity"]
    YawAngle = h5_dict["objects"]["sObject"]["YawAngle"]
    YawRate = h5_dict["objects"]["sObject"]["YawRate"]  
    
    #Write CSV file
    data = {
        'fileTime': fileTime,
        'latAcceleration': latAcceleration,
        'longAcceleration': longAcceleration,
        'brakePedalPos': brakePedalPos,
        'leftDirectionIndicator': leftDirectionIndicator,
        'RightDirectionIndicator': rightDirectionIndicator,
        'steeringAngle': steeringAngle,
        'throttlePedalPos': throttlePedalPos,
        'vehicleSpeed': vehicleSpeed,
        'yawRate': yawRate,
        'egoLaneWidth': egoLaneWidth,
        'yawAngleRightLine': yawAngleRightLine,
        'yawAngleLeftLine': yawAngleLeftLine,
        'curvatureRightLine': curvatureRightLine,
        'curvatureLeftLine': curvatureLeftLine,
        'curvatureDxRightLine': curvatureDxRightLine,
        'curvatureDxLeftLine': curvatureDxLeftLine,
        'rightLaneLine': rightLaneLine,
        'rightLaneLine': rightLaneLine,
        'leftLaneLine': leftLaneLine       
    }

    df = pd.DataFrame(data)

    import numpy as np
    if(np.isnan(LatPosition[0][4])):
        num_vehicles = 2
    else:
        num_vehicles = 11
        
    if(num_vehicles==2): #simpler case with only 1 vehicle
        df[f'car1LatPosition'] = [LatPosition[j][1] for j in range(len(LatPosition))]
        df[f'car1LatVelocity'] = [LatVelocity[j][1] for j in range(len(LatVelocity))]
        df[f'car1LongPosition'] = [LongPosition[j][1] for j in range(len(LongPosition))]
        df[f'car1LongVelocity'] = [LongVelocity[j][1] for j in range(len(LongVelocity))]
        df[f'car1YawAngle'] = [YawAngle[j][1] for j in range(len(YawAngle))]
        df[f'car1YawRate'] = [YawRate[j][1] for j in range(len(YawRate))]
        df[f'car2LatPosition'] = [LatPosition[j][2] for j in range(len(LatPosition))]
        df[f'car2LatVelocity'] = [LatVelocity[j][2] for j in range(len(LatVelocity))]
        df[f'car2LongPosition'] = [LongPosition[j][2] for j in range(len(LongPosition))]
        df[f'car2LongVelocity'] = [LongVelocity[j][2] for j in range(len(LongVelocity))]
        df[f'car2YawAngle'] = [YawAngle[j][2] for j in range(len(YawAngle))]
        df[f'car2YawRate'] = [YawRate[j][2] for j in range(len(YawRate))]
        
    else:
        distances = []
        for i in range(len(LatPosition)):    
            distance = []
            for j in range(1,num_vehicles):
                distance.append(np.sqrt(LatPosition[i][j]**2+LongPosition[i][j]**2))
            distances.append(distance)
        
        #calculate the index of the car with the minimum distance from the ego vehicle for each time step
        minDistancesIndex = []
        for i in range(len(distances)):
            minDistancesIndex.append(np.argmin(distances[i]))
            
        car1LatPosition = []    
        car1LatVelocity= []
        car1LongPosition = []
        car1LongVelocity = []
        car1YawAngle = []
        car1YawRate = []
        
        for k in range(len(minDistancesIndex)):
            car1LatPosition.append(LatPosition[k][minDistancesIndex[k]+1])
            car1LatVelocity.append(LatVelocity[k][minDistancesIndex[k]+1])
            car1LongPosition.append(LongPosition[k][minDistancesIndex[k]+1])
            car1LongVelocity.append(LongVelocity[k][minDistancesIndex[k]+1])
            car1YawAngle.append(YawAngle[k][minDistancesIndex[k]+1])
            car1YawRate.append(YawRate[k][minDistancesIndex[k]+1])
            
        df[f'car1LatPosition'] = car1LatPosition
        df[f'car1LatVelocity'] = car1LatVelocity
        df[f'car1LongPosition'] = car1LongPosition
        df[f'car1LongVelocity'] = car1LongVelocity
        df[f'car1YawAngle'] = car1YawAngle
        df[f'car1YawRate'] = car1YawRate
        
        df[f'car2LatPosition'] = [LatPosition[j][11] for j in range(len(LatPosition))]
        df[f'car2LatVelocity'] = [LatVelocity[j][11] for j in range(len(LatVelocity))]
        df[f'car2LongPosition'] = [LongPosition[j][11] for j in range(len(LongPosition))]
        df[f'car2LongVelocity'] = [LongVelocity[j][11] for j in range(len(LongVelocity))]
        df[f'car2YawAngle'] = [YawAngle[j][11] for j in range(len(YawAngle))]
        df[f'car2YawRate'] = [YawRate[j][11] for j in range(len(YawRate))]
            
    #Initialize labels column
    df['labels'] = 4.1

    #Merge and sort overtake indexes.
    overtakeIndexes = sorted(leftOvertakeIndexes + rightOvertakeIndexes)

    #Update labels based on overtakeIndexes logic.
    for index in overtakeIndexes:
        #Ensure within range of the DataFrame
        if index >= len(df):
            raise ValueError("Index out of range.")        
        
        # Update labels for 40 timesteps before the overtake.
        for offset in range(41):
            step_index = index - offset
            if step_index < 0:
                break
            label_value = round(0.1 * offset, 1)
            # Only update if current label is not '-'.
            if df.at[step_index, 'labels'] != "-":
                df.at[step_index, 'labels'] = label_value
            else:
                break
            
    # Set '-' for the 15 timesteps after the overtake.
        for offset in range(1, 16):  # From 1 to 15 steps after
            step_index = index + offset
            if step_index >= len(df):
                break
            df.at[step_index, 'labels'] = "-"

    #if folder outputs not found create it
    import os
    output_folder = 'dataset'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    # Save the updated DataFrame to a CSV file
    df.to_csv(output_folder+"/"+h5_name+".csv", index=False)