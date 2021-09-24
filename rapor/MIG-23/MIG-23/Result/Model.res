MWS Result File Version 20150206
size=i:15

type=s:HIDDENITEM
problemclass=s::8:1000
visibility=s:hidden
creation=s:internal
lifetime=s:persistent
result=s:0
files=s:MCalcAccess.log

type=s:FOLDER
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:persistent
result=s:0
treepath=s:1D Results

type=s:FOLDER
problemclass=s::8:1000
visibility=s:visible
creation=s:external
lifetime=s:persistent
result=s:0
treepath=s:1D Results\Grafik

type=s:XYSIGNAL2
subtype=s:user
problemclass=s::8:1000
visibility=s:visible
creation=s:external
lifetime=s:persistent
result=s:0
treepath=s:1D Results\Grafik\RCS (square meters),Theta=theta,Phi=phi_(3)
files=s:RCS (square meters),Theta=theta,Phi=phi_1.sig
xlabel=s:Frequency / MHz
title=s:RCS (square meters),Theta=theta,Phi=phi

type=s:XYSIGNAL2
subtype=s:user
problemclass=s::8:1000
visibility=s:visible
creation=s:external
lifetime=s:persistent
result=s:0
treepath=s:1D Results\Grafik\RCS (square meters),Theta=theta,Phi=phi_(4)
files=s:RCS (square meters),Theta=theta,Phi=phi_2.sig
xlabel=s:Frequency / MHz
title=s:RCS (square meters),Theta=theta,Phi=phi

type=s:TABLE
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
treepath=s:Tables\1D Results\RCS (square meters),Theta=theta,Phi=phi
files=s:RCS (square meters),Theta=theta,Phi=phi.rt1
files=s:RCS (square meters),Theta=theta,Phi=phi.rd1

type=s:XYSIGNAL2
subtype=s:time
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\Port signals\Plane wave
files=s:plw.sig
xlabel=s:Time / ns
title=s:Time Signals

type=s:XYSIGNAL2
subtype=s:energy
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\Energy\Energy [pw]
files=s:pw.eng
xlabel=s:Time / ns
title=s:Field Energy / dB

type=s:XYSIGNAL2
subtype=s:complex
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\Power\Excitation [pw]\Power Scattered
files=s:FarfieldMetaData_pw_RadPower.sig
xlabel=s:Frequency / MHz
ylabel=s:W
title=s:Power in Watt

type=s:XYSIGNAL2
subtype=s:linear
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\Cross Sections\Total RCS [pw]
files=s:FarfieldMetaData_pw_TotRCS.sig
xlabel=s:Frequency / MHz
ylabel=s:m2
title=s:Total RCS

type=s:XYSIGNAL2
subtype=s:linear
problemclass=s::8:1000
visibility=s:visible
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\Cross Sections\Total ACS [pw]
files=s:FarfieldMetaData_pw_TotACS.sig
xlabel=s:Frequency / MHz
ylabel=s:m2
title=s:Total ACS

type=s:RESULT_0D
problemclass=s::8:1000
visibility=s:hidden
creation=s:internal
lifetime=s:surviveparchange
result=s:1
parametric=s:P
treepath=s:1D Results\AutomaticRunInformation
files=s:AutomaticRunInformation

type=s:XYSIGNAL2
subtype=s:user
problemclass=s::8:1000
visibility=s:visible
creation=s:external
lifetime=s:persistent
result=s:0
treepath=s:1D Results\Grafik\RCS (square meters),Theta=theta,Phi=phi
files=s:RCS (square meters),Theta=theta,Phi=phi_3.sig
xlabel=s:Frequency / MHz
title=s:RCS (square meters),Theta=theta,Phi=phi

type=s:XYSIGNAL2
subtype=s:user
problemclass=s:Low Frequency:4:3
visibility=s:visible
creation=s:internal
lifetime=s:persistent
result=s:0
treepath=s:Excitation Signals\default
files=s:signal_default_lf.sig
xlabel=s:Time / ns
title=s:Excitation: default

type=s:XYSIGNAL2
subtype=s:user
problemclass=s:High Frequency:0:0
visibility=s:visible
creation=s:internal
lifetime=s:persistent
result=s:0
treepath=s:Excitation Signals\default
files=s:signal_default.sig
xlabel=s:Time / ns
title=s:Excitation: default

