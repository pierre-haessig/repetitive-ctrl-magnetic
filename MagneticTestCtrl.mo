package MagneticTestCtrl "About the control of the voltage of magnetic testbench in order to match a B(t) reference in the magnetic sample. Pierre Haessig 2019"
  extends Modelica.Icons.Package;

  model Experiment
    extends Modelica.Icons.Example;
    MagneticTestCtrl.MagBench magBench annotation(
      Placement(visible = true, transformation(origin = {10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MagneticTestCtrl.MagCtrl magCtrl(f = 50, k = 1, tau = 1e-4)  annotation(
      Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse ref(amplitude = 2, offset = -1, period = 1 / 50)  annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(ref.y, magCtrl.ysp) annotation(
      Line(points = {{-78, 10}, {-70, 10}, {-70, 14}, {-42, 14}, {-42, 14}}, color = {0, 0, 127}));
    connect(magBench.ym, magCtrl.ym) annotation(
      Line(points = {{22, 10}, {40, 10}, {40, -20}, {-60, -20}, {-60, 6}, {-42, 6}, {-42, 6}}, color = {0, 0, 127}));
    connect(magCtrl.u, magBench.u) annotation(
      Line(points = {{-18, 10}, {-2, 10}, {-2, 10}, {-2, 10}}, color = {0, 0, 127}));
  annotation(
      Diagram(graphics = {Text(origin = {-11, 56}, extent = {{-63, 12}, {63, -12}}, textString = "Rep controller with the magnetic experiment")}));end Experiment;

model MagBench
Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance primary(N = 100, i(fixed = false))  "primary winding" annotation(
      Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Magnetic.FluxTubes.Basic.Ground groundMag annotation(
    Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance secondary(N = 100, i(fixed = false))  "secondary winding" annotation(
      Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Electrical.Analog.Sensors.VoltageSensor vSec annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
Modelica.Blocks.Interfaces.RealOutput ym annotation(
    Placement(visible = true, transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealInput u annotation(
    Placement(visible = true, transformation(origin = {-100, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
replaceable Modelica.Magnetic.FluxTubes.Shapes.HysteresisAndMagnets.GenericHystTellinenSoft core( MagRel(fixed = true, start = 0.1))  "core shaped magnetic material under test" annotation(
    Placement(visible = true, transformation(origin = {70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Electrical.Analog.Basic.Resistor r_prim(R = 0.5) "resistance of the primary winding" annotation(
      Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage vPrim annotation(
      Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  equation
    connect(r_prim.n, primary.p) annotation(
      Line(points = {{-10, -20}, {20, -20}, {20, -20}, {20, -20}}, color = {0, 0, 255}));
    connect(vPrim.p, r_prim.p) annotation(
      Line(points = {{-50, -20}, {-30, -20}, {-30, -20}, {-30, -20}}, color = {0, 0, 255}));
    connect(vPrim.n, primary.n) annotation(
      Line(points = {{-50, -40}, {20, -40}}, color = {0, 0, 255}));
    connect(ground.p, vPrim.n) annotation(
      Line(points = {{-50, -60}, {-50, -60}, {-50, -40}, {-50, -40}, {-50, -40}, {-50, -40}}, color = {0, 0, 255}));
    connect(u, vPrim.v) annotation(
      Line(points = {{-100, -30}, {-60, -30}, {-60, -30}, {-58, -30}, {-58, -30}}, color = {0, 0, 127}));
    connect(secondary.port_p, core.port_p) annotation(
      Line(points = {{40, 20}, {70, 20}, {70, 0}}, color = {255, 127, 0}));
    connect(core.port_n, primary.port_n) annotation(
      Line(points = {{70, -20}, {70, -40}, {40, -40}}, color = {255, 127, 0}));
    connect(secondary.port_n, primary.port_p) annotation(
      Line(points = {{40, 0}, {40, -20}}, color = {255, 127, 0}));
    connect(vSec.p, secondary.p) annotation(
      Line(points = {{-50, 20}, {20, 20}}, color = {0, 0, 255}));
    connect(vSec.n, secondary.n) annotation(
      Line(points = {{-50, 0}, {20, 0}}, color = {0, 0, 255}));
    connect(groundMag.port, primary.port_n) annotation(
      Line(points = {{40, -50}, {40, -40}}, color = {255, 127, 0}));
    connect(ground.p, vSec.n) annotation(
      Line(points = {{-50, -60}, {-56, -60}, {-56, -60}, {-60, -60}, {-60, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}}, color = {0, 0, 255}));
    connect(ym, vSec.v) annotation(
      Line(points = {{-30, 40}, {-60, 40}, {-60, 10}}, color = {0, 0, 127}));
    annotation(
      Diagram(graphics = {Text(origin = {100, -12}, extent = {{-20, 20}, {20, -20}}, textString = "core material\nunder test")}));end MagBench;

  model RepCtrl "Repetitive controller"
  
  import Modelica.Constants.pi;
  
  parameter Real k=1 "feedback gain";
  parameter Real ffg=1 "feedforward gain";
  parameter Modelica.SIunits.Frequency f0 "fundamental frequency of the perturbation to reject";
  parameter Modelica.SIunits.Frequency fc "cutoff frequency of the low pass filter in series with the delay (should be >f0)";
  
  Modelica.Blocks.Interfaces.RealInput ym "measurement" annotation(
      Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput ysp "set point" annotation(
      Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput u annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
      Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainFB(k = k)  "feedback gain" annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 1/(2*pi*fc), y(fixed = true))  annotation(
      Placement(visible = true, transformation(origin = {30, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.FixedDelay delay(delayTime = td)  annotation(
      Placement(visible = true, transformation(origin = {-10, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add addFF annotation(
      Placement(visible = true, transformation(origin = {80, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gainFF(k = ffg) "feedforward gain" annotation(
      Placement(visible = true, transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
  parameter Boolean compLag=true "compensate the phase lag introduced by low pass filter in the computation of the delay";
  parameter Modelica.SIunits.Time td = if compLag then 1/f0 - firstOrder.T else 1/f0 "delay time of the repetitive feedback";
  
  equation
    connect(feedback.y, gainFB.u) annotation(
      Line(points = {{-70, 40}, {-68, 40}, {-68, 0}, {-62, 0}, {-62, 0}}, color = {0, 0, 127}));
    connect(gainFF.u, ysp) annotation(
      Line(points = {{-22, 60}, {-96, 60}, {-96, 40}, {-116, 40}, {-116, 40}, {-120, 40}}, color = {0, 0, 127}));
    connect(gainFF.y, addFF.u1) annotation(
      Line(points = {{2, 60}, {60, 60}, {60, 12}, {68, 12}, {68, 12}}, color = {0, 0, 127}));
  connect(addFF.u2, add.y) annotation(
      Line(points = {{68, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {0, 0, 127}));
  connect(addFF.y, u) annotation(
      Line(points = {{92, 6}, {94, 6}, {94, 0}, {110, 0}, {110, 0}}, color = {0, 0, 127}));
    connect(delay.u, firstOrder.y) annotation(
      Line(points = {{2, -50}, {20, -50}, {20, -50}, {18, -50}}, color = {0, 0, 127}));
    connect(delay.y, add.u2) annotation(
      Line(points = {{-21, -50}, {-33, -50}, {-33, -6}, {-21, -6}, {-21, -6}, {-21, -6}, {-21, -6}}, color = {0, 0, 127}));
  connect(gainFB.y, add.u1) annotation(
      Line(points = {{-39, 0}, {-37, 0}, {-37, 0}, {-33, 0}, {-33, 6}, {-21, 6}, {-21, 6}, {-23, 6}, {-23, 6}}, color = {0, 0, 127}));
    connect(firstOrder.u, add.y) annotation(
      Line(points = {{42, -50}, {51, -50}, {51, -50}, {60, -50}, {60, 0}, {2, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {0, 0, 127}));
    connect(ysp, feedback.u1) annotation(
      Line(points = {{-120, 40}, {-88, 40}}, color = {0, 0, 127}));
    connect(ym, feedback.u2) annotation(
      Line(points = {{-120, -40}, {-80, -40}, {-80, 32}}, color = {0, 0, 127}));
  annotation(
      Icon(graphics = {
      Rectangle(fillColor = {255, 254, 209}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}),
      Text(origin = {-2, 60}, extent = {{-80, 20}, {80, -20}}, textString = "Rep. ctrl", textStyle = {TextStyle.Bold}),
      Text(origin = {9, -31}, extent = {{-89, 51}, {71, -49}}, textString = "k=%k
f0=%f0 Hz
fc=%fc Hz", horizontalAlignment = TextAlignment.Left),
      Text(origin = {0, 120}, lineColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name")}, coordinateSystem(initialScale = 0.1)));end RepCtrl;

  package tests
    extends Modelica.Icons.ExamplesPackage;
  
    model testCtrl "test the Repetitive ctrl on a simple system"
      extends Modelica.Icons.Example;
      parameter Modelica.SIunits.Frequency f0=50 "fundamental frequency of the system";
      parameter Real aPert = 1.0 "amplitude of the sinusoidal perturbation";
      parameter Integer n(min=1) = 3 "harmonic of the sinusoidal perturbation";
      parameter Modelica.SIunits.Time startTimePert=0 "startTime of the perturbation";
      parameter Modelica.SIunits.Angle phasePert = 0 "phase of the sinusoidal perturbation";
    
    MagneticTestCtrl.RepCtrl ctrl(f0 = f0, fc = 1e3, k = 1 / 0.8)  "repetitive controller" annotation(
        Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sinePert(amplitude = aPert, freqHz = n * f0, phase = phasePert, startTime = startTimePert) "sinusoidal perturbation on  process input" annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse ref(amplitude = 2, offset = -1, period = 1 / f0) "reference signal" annotation(
        Placement(visible = true, transformation(origin = {-90, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder process(T = 1e-5, k = 0.8, y(fixed = true))  "process being controlled" annotation(
        Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add addPert annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder refFilter(T = 1e-5, k = 1, y(fixed = true)) "filters out the fast fluctuation of the reference that the process cannot reproduce" annotation(
        Placement(visible = true, transformation(origin = {-50, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sinePert.y, addPert.u1) annotation(
        Line(points = {{1, 50}, {8, 50}, {8, 6}, {18, 6}}, color = {0, 0, 127}));
      connect(ref.y, refFilter.u) annotation(
        Line(points = {{-78, 4}, {-62, 4}, {-62, 4}, {-62, 4}}, color = {0, 0, 127}));
  connect(refFilter.y, ctrl.ysp) annotation(
        Line(points = {{-38, 4}, {-22, 4}, {-22, 4}, {-22, 4}}, color = {0, 0, 127}));
  connect(process.y, ctrl.ym) annotation(
        Line(points = {{82, 0}, {90, 0}, {90, -40}, {-32, -40}, {-32, -4}, {-22, -4}}, color = {0, 0, 127}));
  connect(ctrl.u, addPert.u2) annotation(
        Line(points = {{2, 0}, {8, 0}, {8, -6}, {18, -6}, {18, -6}}, color = {0, 0, 127}));
      connect(addPert.y, process.u) annotation(
        Line(points = {{42, 0}, {56, 0}, {56, 0}, {58, 0}}, color = {0, 0, 127}));
      annotation(experiment(StartTime = 0, StopTime = 0.2, Tolerance = 1e-06, Interval = 1e-05),
        Diagram(graphics = {Text(origin = {0, 90}, extent = {{-100, 10}, {100, -10}}, textString = "Test of the repetitive controller\non a simple system")}, coordinateSystem(initialScale = 0.1)));
    end testCtrl;

    model testMagBench
      extends Modelica.Icons.Example;
  MagneticTestCtrl.MagBench magBench annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse pulse(amplitude = 2, offset = -1, period = 1 / 50, startTime = -0.25 / 50)  annotation(
        Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(freqHz = 50)  annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch "switch between sine and pulse inputs" annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant useSine(k = false)  annotation(
        Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(useSine.y, switch.u2) annotation(
        Line(points = {{-38, 0}, {-4, 0}, {-4, 0}, {-2, 0}}, color = {255, 0, 255}));
    connect(pulse.y, switch.u3) annotation(
        Line(points = {{-18, -30}, {-14, -30}, {-14, -8}, {-2, -8}, {-2, -8}}, color = {0, 0, 127}));
    connect(sine.y, switch.u1) annotation(
        Line(points = {{-18, 30}, {-12, 30}, {-12, 8}, {-2, 8}, {-2, 8}}, color = {0, 0, 127}));
    connect(switch.y, magBench.u) annotation(
        Line(points = {{22, 0}, {38, 0}, {38, 0}, {38, 0}}, color = {0, 0, 127}));
    annotation(
        experiment(StartTime = 0, StopTime = 0.1, Tolerance = 1e-6, Interval = 2e-05),
        Diagram(graphics = {Text(origin = {-27, 65}, extent = {{-53, 15}, {107, -5}}, textString = "Test the magnetic system with a square/sine input wave")}));
    end testMagBench;
  end tests;
  annotation(
    uses(Modelica(version = "3.2.2")),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
    Polygon(
      origin={-3.75,0.0},
      fillColor={160,160,164},
      fillPattern=FillPattern.Solid,
      points={{33.75,50.0},{-46.25,50.0},{-46.25,-50.0},{33.75,-50.0},{33.75,-30.0},{-21.25,-30.0},{-21.25,30.0},{33.75,30.0}}),
    Ellipse(
      origin={10.4708,41.6771},
      extent={{-86.0,-24.0},{-78.0,-16.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.0,-20.0},{-78.0,-20.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.1812,-31.6229},{-32.0,-40.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.0,-20.0},{-32.0,-28.0}}),
    Ellipse(
      origin={10.4708,41.6771},
      extent={{-86.0,-60.0},{-78.0,-52.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.0,-56.0},{-78.0,-56.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.0,-44.0},{-32.0,-52.0}}),
    Line(
      origin={10.4708,41.6771},
      points={{-64.0,-56.0},{-32.0,-64.0}}),
    Rectangle(
      origin={62.5,0.0},
      fillColor={160,160,164},
      fillPattern=FillPattern.Solid,
      extent={{-12.5,-50.0},{12.5,50.0}})}));
end MagneticTestCtrl;