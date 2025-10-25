from FLUFFY.conveyors import conveyor

conveyorList=[]
conveyorList.append(conveyor(conveyorSelect= "LongConveyor", debugMode= True))
conveyorList.append(conveyor(conveyorSelect= "SideConveyor_01", debugMode= True))
conveyorList.append(conveyor(conveyorSelect= "SideConveyor_02", debugMode= True))
conveyorList.append(conveyor(conveyorSelect= "PistonBelt_01", debugMode= True))
conveyorList.append(conveyor(conveyorSelect= "PistonBelt_02", debugMode= True))

direction = [0, 1, 0]

for conveyor in conveyorList:
    conveyor.enableBelt()