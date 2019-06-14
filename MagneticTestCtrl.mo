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
  Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance primary(N = 100, i(fixed = true, start = 0))  annotation(
      Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Magnetic.FluxTubes.Basic.Ground groundMag annotation(
      Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance secondary(N = 100)  annotation(
      Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor vSec annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.SignalVoltage vPrim annotation(
      Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput ym annotation(
      Placement(visible = true, transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-100, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  replaceable Modelica.Magnetic.FluxTubes.Shapes.HysteresisAndMagnets.GenericHystTellinenSoft testCore( MagRel(fixed = true), Phi(fixed = true, start = 0))  "core shaped magnetic material under test" annotation(
      Placement(visible = true, transformation(origin = {70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor Rpar(R = 1e3)  "parallel resistor" annotation(
      Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Resistor r_prim(R = 1) annotation(
      Placement(visible = true, transformation(origin = {-26, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
    connect(secondary.port_p, testCore.port_p) annotation(
      Line(points = {{40, 20}, {70, 20}, {70, 0}}, color = {255, 127, 0}));
    connect(testCore.port_n, primary.port_n) annotation(
      Line(points = {{70, -20}, {70, -40}, {40, -40}}, color = {255, 127, 0}));
    connect(Rpar.p, primary.p) annotation(
      Line(points = {{0, -20}, {20, -20}, {20, -20}, {20, -20}}, color = {0, 0, 255}));
    connect(r_prim.p, Rpar.p) annotation(
      Line(points = {{-16, -20}, {0, -20}, {0, -20}, {0, -20}}, color = {0, 0, 255}));
    connect(vPrim.p, r_prim.n) annotation(
      Line(points = {{-50, -20}, {-36, -20}, {-36, -20}, {-36, -20}}, color = {0, 0, 255}));
    connect(vPrim.n, Rpar.n) annotation(
      Line(points = {{-50, -40}, {0, -40}}, color = {0, 0, 255}));
    connect(secondary.port_n, primary.port_p) annotation(
      Line(points = {{40, 0}, {40, -20}}, color = {255, 127, 0}));
    connect(vSec.p, secondary.p) annotation(
      Line(points = {{-50, 20}, {20, 20}}, color = {0, 0, 255}));
    connect(vSec.n, secondary.n) annotation(
      Line(points = {{-50, 0}, {20, 0}}, color = {0, 0, 255}));
    connect(groundMag.port, primary.port_n) annotation(
      Line(points = {{40, -50}, {40, -40}}, color = {255, 127, 0}));
    connect(vPrim.n, primary.n) annotation(
      Line(points = {{-50, -40}, {20, -40}}, color = {0, 0, 255}));
  connect(ground.p, vSec.n) annotation(
      Line(points = {{-50, -60}, {-56, -60}, {-56, -60}, {-60, -60}, {-60, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}, {-50, 2.98024e-07}}, color = {0, 0, 255}));
  connect(ground.p, vPrim.n) annotation(
      Line(points = {{-50, -60}, {-50, -60}, {-50, -40}, {-50, -40}, {-50, -40}, {-50, -40}}, color = {0, 0, 255}));
  connect(u, vPrim.v) annotation(
      Line(points = {{-100, -30}, {-60, -30}, {-60, -30}, {-58, -30}, {-58, -30}}, color = {0, 0, 127}));
  connect(ym, vSec.v) annotation(
      Line(points = {{-30, 40}, {-60, 40}, {-60, 10}}, color = {0, 0, 127}));
  end MagBench;

  model MagCtrl
  
  parameter Real k "feedback gain";
  parameter Modelica.SIunits.Frequency f "frequency of the perturbation to reject";
  parameter Modelica.SIunits.Time tau "time constant of first order low pass filter (should be a priori <T)";
  
  protected
  parameter Modelica.SIunits.Time td = 1/f-tau "delay time";
  
  
  Modelica.Blocks.Interfaces.RealInput ym "measurement" annotation(
      Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput ysp "set point" annotation(
      Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput u annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
      Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = k)  annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = tau, y(fixed = true))  annotation(
      Placement(visible = true, transformation(origin = {30, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.FixedDelay delay(delayTime = td)  annotation(
      Placement(visible = true, transformation(origin = {-10, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add addFF annotation(
      Placement(visible = true, transformation(origin = {80, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(addFF.u1, ysp) annotation(
      Line(points = {{68, 12}, {60, 12}, {60, 60}, {-96, 60}, {-96, 40}, {-120, 40}, {-120, 40}}, color = {0, 0, 127}));
  connect(addFF.u2, add.y) annotation(
      Line(points = {{68, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {0, 0, 127}));
  connect(addFF.y, u) annotation(
      Line(points = {{92, 6}, {94, 6}, {94, 0}, {110, 0}, {110, 0}}, color = {0, 0, 127}));
    connect(feedback.y, gain.u) annotation(
      Line(points = {{-71, 40}, {-62, 40}, {-62, 0}}, color = {0, 0, 127}));
    connect(delay.u, firstOrder.y) annotation(
      Line(points = {{2, -50}, {20, -50}, {20, -50}, {18, -50}}, color = {0, 0, 127}));
    connect(delay.y, add.u2) annotation(
      Line(points = {{-21, -50}, {-33, -50}, {-33, -6}, {-21, -6}, {-21, -6}, {-21, -6}, {-21, -6}}, color = {0, 0, 127}));
    connect(gain.y, add.u1) annotation(
      Line(points = {{-39, 0}, {-37, 0}, {-37, 0}, {-33, 0}, {-33, 6}, {-21, 6}, {-21, 6}, {-23, 6}, {-23, 6}}, color = {0, 0, 127}));
    connect(firstOrder.u, add.y) annotation(
      Line(points = {{42, -50}, {51, -50}, {51, -50}, {60, -50}, {60, 0}, {2, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {0, 0, 127}));
    connect(ysp, feedback.u1) annotation(
      Line(points = {{-120, 40}, {-88, 40}}, color = {0, 0, 127}));
    connect(ym, feedback.u2) annotation(
      Line(points = {{-120, -40}, {-80, -40}, {-80, 32}}, color = {0, 0, 127}));
  annotation(
      Icon(graphics = {Text(origin = {-2, 60}, extent = {{-80, 20}, {80, -20}}, textString = "Rep. ctrl"), Text(origin = {9, -31}, extent = {{-71, 49}, {71, -49}}, textString = "f=%f\nk=%k\nτ=%tau", horizontalAlignment = TextAlignment.Left), Rectangle(extent = {{-100, 100}, {100, -100}})}));end MagCtrl;

  package tests
    extends Modelica.Icons.ExamplesPackage;
  
    model testCtrl "test the Repetitive ctrl on a simplified system"
      extends Modelica.Icons.Example;
    
    MagneticTestCtrl.MagCtrl magCtrl(f = 50,k = 1, tau = 1e-4)  annotation(
        Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sineF1(freqHz = 50, phase = 1.5708)  annotation(
        Placement(visible = true, transformation(origin = {-50, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sineF3(freqHz = 150)  annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse ref(amplitude = 2, offset = -1, period = 1 / 50) "reference signal" annotation(
        Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 1e-3, k = 0.8, y(fixed = true))  annotation(
        Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add addSin annotation(
        Placement(visible = true, transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add addPert annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(firstOrder.y, magCtrl.ym) annotation(
        Line(points = {{82, 0}, {90, 0}, {90, -40}, {-32, -40}, {-32, -4}, {-22, -4}}, color = {0, 0, 127}));
      connect(addSin.y, addPert.u1) annotation(
        Line(points = {{2, 60}, {8, 60}, {8, 6}, {18, 6}, {18, 6}}, color = {0, 0, 127}));
      connect(magCtrl.u, addPert.u2) annotation(
        Line(points = {{2, 0}, {8, 0}, {8, -6}, {18, -6}, {18, -6}}, color = {0, 0, 127}));
      connect(addPert.y, firstOrder.u) annotation(
        Line(points = {{42, 0}, {56, 0}, {56, 0}, {58, 0}}, color = {0, 0, 127}));
    connect(sineF3.y, addSin.u2) annotation(
        Line(points = {{-39, 40}, {-37, 40}, {-37, 40}, {-33, 40}, {-33, 54}, {-21, 54}, {-21, 55}, {-23, 55}, {-23, 54}}, color = {0, 0, 127}));
    connect(sineF1.y, addSin.u1) annotation(
        Line(points = {{-39, 78}, {-33, 78}, {-33, 66}, {-23, 66}}, color = {0, 0, 127}));
      connect(ref.y, magCtrl.ysp) annotation(
        Line(points = {{-38, 0}, {-36, 0}, {-36, 4}, {-22, 4}, {-22, 4}}, color = {0, 0, 127}));
    annotation(experiment(StartTime=0,StopTime=0.2));
    end testCtrl;

    model testMagBench
      extends Modelica.Icons.Example;
  MagBench magBench annotation(
        Placement(visible = true, transformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse pulse(amplitude = 2, offset = -1, period = 1 / 50)  annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(pulse.y, magBench.u) annotation(
        Line(points = {{-16, 0}, {18, 0}, {18, 0}, {20, 0}}, color = {0, 0, 127}));
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