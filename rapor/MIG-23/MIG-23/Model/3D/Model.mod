'# MWS Version: Version 2016.0 - Jan 22 2016 - ACIS 25.0.2 -

'# length = m
'# frequency = MHz
'# time = ns
'# frequency range: fmin = 10 fmax = 125
'# created = '[VERSION]2016.0|25.0.2|20160122[/VERSION]


'@ use template: Mono-static RCS - Small Object_4.cfg

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
'set the units
With Units
    .Geometry "m"
    .Frequency "MHz"
    .Voltage "V"
    .Resistance "Ohm"
    .Inductance "NanoH"
    .TemperatureUnit  "Kelvin"
    .Time "ns"
    .Current "A"
    .Conductance "Siemens"
    .Capacitance "PikoF"
End With
'----------------------------------------------------------------------------
Plot.DrawBox True
With Background
     .Type "Normal"
     .Epsilon "1.0"
     .Mue "1.0"
     .Rho "1.204"
     .ThermalType "Normal"
     .ThermalConductivity "0.026"
     .HeatCapacity "1.005"
     .XminSpace "0.0"
     .XmaxSpace "0.0"
     .YminSpace "0.0"
     .YmaxSpace "0.0"
     .ZminSpace "0.0"
     .ZmaxSpace "0.0"
End With
With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "expanded open"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
End With
MakeSureParameterExists "alpha", "0"
SetParameterDescription "alpha", "Rotation angle of E-field Vector (relative to x-axis)"
MakeSureParameterExists "theta", "0"
SetParameterDescription "theta", "spherical angle of incident plane wave"
MakeSureParameterExists "phi", "0"
SetParameterDescription "phi", "spherical angle of incident plane wave"
With PlaneWave
     .Reset
     .Normal "-sinD(theta)*cosD(phi)", "-sinD(theta)*sinD(phi)", "-cosD(theta)"
     .EVector "cosD(phi)*cosD(theta)*cosD(alpha)-sinD(phi)*sinD(alpha)", "sinD(phi)*cosD(theta)*cosD(alpha)+cosD(phi)*sinD(alpha)", "-sinD(theta)*cosD(alpha)"
     .Polarization "Linear"
     .ReferenceFrequency "0.0"
     .PhaseDifference "-90.0"
     .CircularDirection "Left"
     .AxialRatio "1.0"
     .SetUserDecouplingPlane "False"
     .Store
End With
Mesh.FPBAAvoidNonRegUnite "True"
With MeshSettings
     .SetMeshType "Srf"
     .Set "Version", 1
End With
IESolver.SetCFIEAlpha "1.000000"
'----------------------------------------------------------------------------
With MeshSettings
     .SetMeshType "Hex"
     .Set "Version", 1%
End With
With Mesh
     .MeshType "PBA"
End With
'set the solver type
ChangeSolverType("HF Time Domain")

'@ new component: component1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Component.New "component1"

'@ define sphere: component1:solid1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Sphere 
     .Reset 
     .Name "solid1" 
     .Component "component1" 
     .Material "PEC" 
     .Axis "z" 
     .CenterRadius "5" 
     .TopRadius "2.5" 
     .BottomRadius "0" 
     .Center "0", "0", "0" 
     .Segments "0" 
     .Create 
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ delete shape: component1:solid1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.Delete "component1:solid1"

'@ define cylinder: component1:solid1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Cylinder 
     .Reset 
     .Name "solid1" 
     .Component "component1" 
     .Material "PEC" 
     .OuterRadius "5" 
     .InnerRadius "0.0" 
     .Axis "z" 
     .Zrange "0", "L/4" 
     .Xcenter "-L/2" 
     .Ycenter "L/2" 
     .Segments "0" 
     .Create 
End With

'@ delete shape: component1:solid1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.Delete "component1:solid1"

'@ define cylinder: component1:solid1

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Cylinder 
     .Reset 
     .Name "solid1" 
     .Component "component1" 
     .Material "PEC" 
     .OuterRadius "r" 
     .InnerRadius "0.0" 
     .Axis "x" 
     .Xrange "-L/2+h_koni", "L/2" 
     .Ycenter "0" 
     .Zcenter "0" 
     .Segments "0" 
     .Create 
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define brick: component1:solid2

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Brick
     .Reset 
     .Name "solid2" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-1", "1" 
     .Yrange "-w/2", "w/2" 
     .Zrange "-0.1", "0.1" 
     .Create
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "1200"

'@ define time domain solver parameters

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Mesh.SetCreator "High Frequency" 
With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-PLW"
     .StimulationPort "Plane wave"
     .StimulationMode "1"
     .SteadyStateLimit "-30.0"
     .MeshAdaption "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "0", "1"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "(3e8/Lmax)"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ change material: component1:solid1 to: PEC

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.ChangeMaterial "component1:solid1", "PEC"

'@ change material: component1:solid2 to: PEC

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.ChangeMaterial "component1:solid2", "PEC"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ set mesh properties (Hexahedral)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Mesh 
     .MeshType "PBA" 
     .SetCreator "High Frequency"
End With 
With MeshSettings 
     .SetMeshType "Hex" 
     .Set "Version", 1%
     'MAX CELL - WAVELENGTH REFINEMENT 
     .Set "StepsPerWaveNear", "15" 
     .Set "StepsPerWaveFar", "15" 
     .Set "WavelengthRefinementSameAsNear", "1" 
     'MAX CELL - GEOMETRY REFINEMENT 
     .Set "StepsPerBoxNear", "20" 
     .Set "StepsPerBoxFar", "1" 
     .Set "MaxStepNear", "0" 
     .Set "MaxStepFar", "0" 
     .Set "ModelBoxDescrNear", "maxedge" 
     .Set "ModelBoxDescrFar", "maxedge" 
     .Set "UseMaxStepAbsolute", "0" 
     .Set "GeometryRefinementSameAsNear", "0" 
     'MIN CELL 
     .Set "UseRatioLimitGeometry", "1" 
     .Set "RatioLimitGeometry", "15" 
     .Set "MinStepGeometryX", "0" 
     .Set "MinStepGeometryY", "0" 
     .Set "MinStepGeometryZ", "0" 
     .Set "UseSameMinStepGeometryXYZ", "1" 
End With 
With MeshSettings 
     .Set "PlaneMergeVersion", "2" 
End With 
With MeshSettings 
     .SetMeshType "Hex" 
     .Set "FaceRefinementOn", "0" 
     .Set "FaceRefinementPolicy", "2" 
     .Set "FaceRefinementRatio", "2" 
     .Set "FaceRefinementStep", "0" 
     .Set "FaceRefinementNSteps", "2" 
     .Set "EllipseRefinementOn", "0" 
     .Set "EllipseRefinementPolicy", "2" 
     .Set "EllipseRefinementRatio", "2" 
     .Set "EllipseRefinementStep", "0" 
     .Set "EllipseRefinementNSteps", "2" 
     .Set "FaceRefinementBufferLines", "3" 
     .Set "EdgeRefinementOn", "1" 
     .Set "EdgeRefinementPolicy", "1" 
     .Set "EdgeRefinementRatio", "2" 
     .Set "EdgeRefinementStep", "0" 
     .Set "EdgeRefinementBufferLines", "3" 
     .Set "RefineEdgeMaterialGlobal", "0" 
     .Set "RefineAxialEdgeGlobal", "0" 
     .Set "BufferLinesNear", "3" 
     .Set "UseDielectrics", "1" 
     .Set "EquilibrateOn", "0" 
     .Set "Equilibrate", "1.5" 
     .Set "IgnoreThinPanelMaterial", "0" 
End With 
With MeshSettings 
     .SetMeshType "Hex" 
     .Set "SnapToAxialEdges", "1"
     .Set "SnapToPlanes", "1"
     .Set "SnapToSpheres", "1"
     .Set "SnapToEllipses", "1"
     .Set "SnapToCylinders", "1"
     .Set "SnapToCylinderCenters", "1"
     .Set "SnapToEllipseCenters", "1"
End With 
With Discretizer 
     .MeshType "PBA" 
     .PBAType "Fast PBA" 
     .AutomaticPBAType "True" 
     .FPBAAccuracyEnhancement "enable"
     .ConnectivityCheck "False"
     .ConvertGeometryDataAfterMeshing "True" 
     .UsePecEdgeModel "True" 
     .GapDetection "False" 
     .FPBAGapTolerance "1e-3" 
     .PointAccEnhancement "0" 
     .UseSplitComponents "True" 
     .EnableSubgridding "False" 
     .PBAFillLimit "99" 
     .AlwaysExcludePec "False" 
End With

'@ define cone: component1:solid3

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Cone 
     .Reset 
     .Name "solid3" 
     .Component "component1" 
     .Material "PEC" 
     .BottomRadius "r" 
     .TopRadius "0.0" 
     .Axis "x" 
     .Xrange "-L/2", "-L/2+h_koni" 
     .Ycenter "0" 
     .Zcenter "0" 
     .Segments "0" 
     .Create 
End With

'@ transform: mirror component1:solid3

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Transform 
     .Reset 
     .Name "component1:solid3" 
     .Origin "CommonCenter" 
     .Center "0", "0", "0" 
     .PlaneNormal "1", "0", "0" 
     .MultipleObjects "False" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Transform "Shape", "Mirror" 
End With

'@ change material: component1:solid3 to: PEC

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.ChangeMaterial "component1:solid3", "PEC"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define brick: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Brick
     .Reset 
     .Name "solid4" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-L/2", "L/2" 
     .Yrange "-4", "4" 
     .Zrange "5", "6" 
     .Create
End With

'@ delete shape: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.Delete "component1:solid4"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ pick edge

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEdgeFromId "component1:solid2", "1", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "0"
    .SetOrientation "Smart Mode"
    .SetDistance "10.442969"
    .SetViewVector "-0.408348", "-0.811481", "-0.418031"
    .Create
End With
Pick.ClearAllPicks

'@ delete dimension 0

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .RemoveDimension "0"
End With

'@ define sphere: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Sphere 
     .Reset 
     .Name "solid4" 
     .Component "component1" 
     .Material "PEC" 
     .Axis "z" 
     .CenterRadius "0.25" 
     .TopRadius "0" 
     .BottomRadius "0" 
     .Center "0", "0", "0" 
     .Segments "0" 
     .Create 
End With

'@ delete shape: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.Delete "component1:solid4"

'@ switch bounding box

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawBox "True"

'@ define units

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Units 
     .Geometry "m" 
     .Frequency "Hz" 
     .Time "ns" 
     .TemperatureUnit "Kelvin" 
     .Voltage "V" 
     .Current "A" 
     .Resistance "Ohm" 
     .Conductance "Siemens" 
     .Capacitance "PikoF" 
     .Inductance "NanoH" 
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "(3e8/Lmax)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10000000", "5.35714285714286e+007", "20e+6"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10000000)" 
Monitor.Delete "farfield (f=30000000)" 
Monitor.Delete "farfield (f=50000000)"

'@ define monitor: e-field (f=2.67857192857143e+007)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor 
     .Reset 
     .Name "e-field (f=2.67857192857143e+007)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Efield" 
     .Frequency "2.67857192857143e+007" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .Create 
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define brick: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Brick
     .Reset 
     .Name "solid4" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-l/2", "l/2" 
     .Yrange "-15", "-20" 
     .Zrange "0", "3" 
     .Create
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ delete shape: component1:solid4

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solid.Delete "component1:solid4"

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define units

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Units 
     .Geometry "m" 
     .Frequency "MHz" 
     .Time "ns" 
     .TemperatureUnit "Kelvin" 
     .Voltage "V" 
     .Current "A" 
     .Resistance "Ohm" 
     .Conductance "Siemens" 
     .Capacitance "PikoF" 
     .Inductance "NanoH" 
End With

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "5", "55", "5"
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "55"

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "72"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "72", "2"
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "100"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "100", "5"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "120"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "12", "120", "10"
End With

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "12", "120", "11.5"
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "e-field (f=2.67857192857143e+007)" 
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=115.5)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=15)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=23.5)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=25)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=35)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=45)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=46.5)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=5)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=55)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=65)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=69.5)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=75)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=81)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=85)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=92.5)" 
Monitor.Delete "farfield (f=95)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "12", "120", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "120"

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "125"

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "12.5", "125", "12"
End With

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "12.5", "125", "2"
End With

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=100.5)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=102.5)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=104.5)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=106.5)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=108.5)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=110.5)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=112.5)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=114.5)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=116.5)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=118.5)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=12.5)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=120.5)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=122.5)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=124.5)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=14.5)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=16.5)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=18.5)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=20.5)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=22.5)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=24.5)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=26.5)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=28.5)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=30.5)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=32.5)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=34.5)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=36.5)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=38.5)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=40.5)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=42.5)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=44.5)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=46.5)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=48.5)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=50.5)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=52.5)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=54.5)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=56.5)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=58.5)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=60.5)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=62.5)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=64.5)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=66.5)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=68.5)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=70.5)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=72.5)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=74.5)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=76.5)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=78.5)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=80.5)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=82.5)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=84.5)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=86.5)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=88.5)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=90.5)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=92.5)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=94.5)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=96.5)" 
Monitor.Delete "farfield (f=98)" 
Monitor.Delete "farfield (f=98.5)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Efield"
          .Dimension "Volume" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "e-field (f=10)" 
Monitor.Delete "e-field (f=100)" 
Monitor.Delete "e-field (f=102)" 
Monitor.Delete "e-field (f=104)" 
Monitor.Delete "e-field (f=106)" 
Monitor.Delete "e-field (f=108)" 
Monitor.Delete "e-field (f=110)" 
Monitor.Delete "e-field (f=112)" 
Monitor.Delete "e-field (f=114)" 
Monitor.Delete "e-field (f=116)" 
Monitor.Delete "e-field (f=118)" 
Monitor.Delete "e-field (f=12)" 
Monitor.Delete "e-field (f=120)" 
Monitor.Delete "e-field (f=122)" 
Monitor.Delete "e-field (f=124)" 
Monitor.Delete "e-field (f=14)" 
Monitor.Delete "e-field (f=16)" 
Monitor.Delete "e-field (f=18)" 
Monitor.Delete "e-field (f=20)" 
Monitor.Delete "e-field (f=22)" 
Monitor.Delete "e-field (f=24)" 
Monitor.Delete "e-field (f=26)" 
Monitor.Delete "e-field (f=28)" 
Monitor.Delete "e-field (f=30)" 
Monitor.Delete "e-field (f=32)" 
Monitor.Delete "e-field (f=34)" 
Monitor.Delete "e-field (f=36)" 
Monitor.Delete "e-field (f=38)" 
Monitor.Delete "e-field (f=40)" 
Monitor.Delete "e-field (f=42)" 
Monitor.Delete "e-field (f=44)" 
Monitor.Delete "e-field (f=46)" 
Monitor.Delete "e-field (f=48)" 
Monitor.Delete "e-field (f=50)" 
Monitor.Delete "e-field (f=52)" 
Monitor.Delete "e-field (f=54)" 
Monitor.Delete "e-field (f=56)" 
Monitor.Delete "e-field (f=58)" 
Monitor.Delete "e-field (f=60)" 
Monitor.Delete "e-field (f=62)" 
Monitor.Delete "e-field (f=64)" 
Monitor.Delete "e-field (f=66)" 
Monitor.Delete "e-field (f=68)" 
Monitor.Delete "e-field (f=70)" 
Monitor.Delete "e-field (f=72)" 
Monitor.Delete "e-field (f=74)" 
Monitor.Delete "e-field (f=76)" 
Monitor.Delete "e-field (f=78)" 
Monitor.Delete "e-field (f=80)" 
Monitor.Delete "e-field (f=82)" 
Monitor.Delete "e-field (f=84)" 
Monitor.Delete "e-field (f=86)" 
Monitor.Delete "e-field (f=88)" 
Monitor.Delete "e-field (f=90)" 
Monitor.Delete "e-field (f=92)" 
Monitor.Delete "e-field (f=94)" 
Monitor.Delete "e-field (f=96)" 
Monitor.Delete "e-field (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "125"

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "5", "125"

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid1", "2", "3", "1"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEndpointFromId "component1:solid3", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "0"
    .SetOrientation "Smart Mode"
    .SetDistance "3.100428"
    .SetViewVector "0.000000", "-0.000095", "-1.000000"
    .Create
End With
Pick.ClearAllPicks

'@ delete dimension 0

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .RemoveDimension "0"
End With

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid1", "1", "1", "1"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid1", "2", "3", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "1"
    .SetOrientation "Smart Mode"
    .SetDistance "-2.495182"
    .SetViewVector "0.000000", "-0.000095", "-1.000000"
    .Create
End With
Pick.ClearAllPicks

'@ switch working plane

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawWorkplane "false"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEndpointFromId "component1:solid3", "2"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEndpointFromId "component1:solid3", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "2"
    .SetOrientation "Smart Mode"
    .SetDistance "1.879622"
    .SetViewVector "-0.003491", "-0.000062", "-0.999994"
    .Create
End With
Pick.ClearAllPicks

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ pick edge

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEdgeFromId "component1:solid2", "1", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "3"
    .SetOrientation "Smart Mode"
    .SetDistance "2.050261"
    .SetViewVector "0.000000", "-0.000095", "-1.000000"
    .Create
End With
Pick.ClearAllPicks

'@ switch working plane

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawWorkplane "true"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "54" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "54" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Efield"
          .Dimension "Volume" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "e-field (f=10)" 
Monitor.Delete "e-field (f=100)" 
Monitor.Delete "e-field (f=102)" 
Monitor.Delete "e-field (f=104)" 
Monitor.Delete "e-field (f=106)" 
Monitor.Delete "e-field (f=108)" 
Monitor.Delete "e-field (f=110)" 
Monitor.Delete "e-field (f=112)" 
Monitor.Delete "e-field (f=114)" 
Monitor.Delete "e-field (f=116)" 
Monitor.Delete "e-field (f=118)" 
Monitor.Delete "e-field (f=12)" 
Monitor.Delete "e-field (f=120)" 
Monitor.Delete "e-field (f=122)" 
Monitor.Delete "e-field (f=124)" 
Monitor.Delete "e-field (f=14)" 
Monitor.Delete "e-field (f=16)" 
Monitor.Delete "e-field (f=18)" 
Monitor.Delete "e-field (f=20)" 
Monitor.Delete "e-field (f=22)" 
Monitor.Delete "e-field (f=24)" 
Monitor.Delete "e-field (f=26)" 
Monitor.Delete "e-field (f=28)" 
Monitor.Delete "e-field (f=30)" 
Monitor.Delete "e-field (f=32)" 
Monitor.Delete "e-field (f=34)" 
Monitor.Delete "e-field (f=36)" 
Monitor.Delete "e-field (f=38)" 
Monitor.Delete "e-field (f=40)" 
Monitor.Delete "e-field (f=42)" 
Monitor.Delete "e-field (f=44)" 
Monitor.Delete "e-field (f=46)" 
Monitor.Delete "e-field (f=48)" 
Monitor.Delete "e-field (f=50)" 
Monitor.Delete "e-field (f=52)" 
Monitor.Delete "e-field (f=54)" 
Monitor.Delete "e-field (f=56)" 
Monitor.Delete "e-field (f=58)" 
Monitor.Delete "e-field (f=60)" 
Monitor.Delete "e-field (f=62)" 
Monitor.Delete "e-field (f=64)" 
Monitor.Delete "e-field (f=66)" 
Monitor.Delete "e-field (f=68)" 
Monitor.Delete "e-field (f=70)" 
Monitor.Delete "e-field (f=72)" 
Monitor.Delete "e-field (f=74)" 
Monitor.Delete "e-field (f=76)" 
Monitor.Delete "e-field (f=78)" 
Monitor.Delete "e-field (f=80)" 
Monitor.Delete "e-field (f=82)" 
Monitor.Delete "e-field (f=84)" 
Monitor.Delete "e-field (f=86)" 
Monitor.Delete "e-field (f=88)" 
Monitor.Delete "e-field (f=90)" 
Monitor.Delete "e-field (f=92)" 
Monitor.Delete "e-field (f=94)" 
Monitor.Delete "e-field (f=96)" 
Monitor.Delete "e-field (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "54" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "54" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ switch working plane

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawWorkplane "false"

'@ switch bounding box

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawBox "True"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickEndpointFromId "component1:solid3", "2"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid1", "2", "3", "1"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "4"
    .SetOrientation "Smart Mode"
    .SetDistance "-2.235695"
    .SetViewVector "0.000000", "1.000000", "-0.000035"
    .Create
End With
Pick.ClearAllPicks

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid3", "1", "2", "0"

'@ pick end point

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.PickExtraCirclepointFromId "component1:solid3", "1", "2", "2"

'@ define distance dimension by picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Dimension
    .Reset
    .UsePicks True
    .SetType "Distance"
    .SetID "5"
    .SetOrientation "Smart Mode"
    .SetDistance "-1.463450"
    .SetViewVector "1.000000", "-0.000000", "0.000030"
    .Create
End With
Pick.ClearAllPicks

'@ clear picks

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Pick.ClearAllPicks

'@ switch bounding box

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawBox "False"

'@ delete monitors

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Monitor.Delete "farfield (f=10)" 
Monitor.Delete "farfield (f=100)" 
Monitor.Delete "farfield (f=102)" 
Monitor.Delete "farfield (f=104)" 
Monitor.Delete "farfield (f=106)" 
Monitor.Delete "farfield (f=108)" 
Monitor.Delete "farfield (f=110)" 
Monitor.Delete "farfield (f=112)" 
Monitor.Delete "farfield (f=114)" 
Monitor.Delete "farfield (f=116)" 
Monitor.Delete "farfield (f=118)" 
Monitor.Delete "farfield (f=12)" 
Monitor.Delete "farfield (f=120)" 
Monitor.Delete "farfield (f=122)" 
Monitor.Delete "farfield (f=124)" 
Monitor.Delete "farfield (f=14)" 
Monitor.Delete "farfield (f=16)" 
Monitor.Delete "farfield (f=18)" 
Monitor.Delete "farfield (f=20)" 
Monitor.Delete "farfield (f=22)" 
Monitor.Delete "farfield (f=24)" 
Monitor.Delete "farfield (f=26)" 
Monitor.Delete "farfield (f=28)" 
Monitor.Delete "farfield (f=30)" 
Monitor.Delete "farfield (f=32)" 
Monitor.Delete "farfield (f=34)" 
Monitor.Delete "farfield (f=36)" 
Monitor.Delete "farfield (f=38)" 
Monitor.Delete "farfield (f=40)" 
Monitor.Delete "farfield (f=42)" 
Monitor.Delete "farfield (f=44)" 
Monitor.Delete "farfield (f=46)" 
Monitor.Delete "farfield (f=48)" 
Monitor.Delete "farfield (f=50)" 
Monitor.Delete "farfield (f=52)" 
Monitor.Delete "farfield (f=54)" 
Monitor.Delete "farfield (f=56)" 
Monitor.Delete "farfield (f=58)" 
Monitor.Delete "farfield (f=60)" 
Monitor.Delete "farfield (f=62)" 
Monitor.Delete "farfield (f=64)" 
Monitor.Delete "farfield (f=66)" 
Monitor.Delete "farfield (f=68)" 
Monitor.Delete "farfield (f=70)" 
Monitor.Delete "farfield (f=72)" 
Monitor.Delete "farfield (f=74)" 
Monitor.Delete "farfield (f=76)" 
Monitor.Delete "farfield (f=78)" 
Monitor.Delete "farfield (f=80)" 
Monitor.Delete "farfield (f=82)" 
Monitor.Delete "farfield (f=84)" 
Monitor.Delete "farfield (f=86)" 
Monitor.Delete "farfield (f=88)" 
Monitor.Delete "farfield (f=90)" 
Monitor.Delete "farfield (f=92)" 
Monitor.Delete "farfield (f=94)" 
Monitor.Delete "farfield (f=96)" 
Monitor.Delete "farfield (f=98)"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

'@ farfield plot options

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "0" 
     .Phi "0" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "54" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "RCSUNITS" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+000", "0.000000e+000", "0.000000e+000" 
     .Thetastart "0.000000e+000", "0.000000e+000", "1.000000e+000" 
     .PolarizationVector "0.000000e+000", "1.000000e+000", "0.000000e+000" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+000 
     .Origin "bbox" 
     .Userorigin "0.000000e+000", "0.000000e+000", "0.000000e+000" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+000" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+001" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .StoreSettings
End With

'@ switch bounding box

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawBox "True"

'@ switch working plane

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Plot.DrawWorkplane "true"

'@ define frequency range

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
Solver.FrequencyRange "10", "125"

'@ define monitors (using linear step)

'[VERSION]2016.0|25.0.2|20160122[/VERSION]
With Monitor
          .Reset 
          .Domain "Frequency"
          .FieldType "Farfield"
          .ExportFarfieldSource "False" 
          .UseSubvolume "False" 
          .Coordinates "Structure" 
          .SetSubvolume "-8.4", "8.4", "-7.125", "7.125", "-1.0875", "1.0875" 
          .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
          .SetSubvolumeOffsetType "FractionOfWavelength" 
          .CreateUsingLinearStep "10", "125", "2"
End With

