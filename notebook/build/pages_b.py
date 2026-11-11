"""Section G pages, part 2: brain GUI, the autotuners, the asterisk drive, macros and telemetry."""
import json
import os

from nb import B, BOX, CODE, H, IMGB, MATRIX, P, SIDE, TABLE, Page, GREEN, RED, YELLOW, GRAY, BLUE
from pages_a import ASTROM, EZ, LEMLIB, PROS, GM0, O, G

HERE = os.path.dirname(os.path.abspath(__file__))


def data(name):
    with open(os.path.join(HERE, 'data', name + '.json'), encoding='utf-8') as f:
        return json.load(f)


AT = data('autotuners')
LVGL = 'LVGL 9.2 docs, docs.lvgl.io/9.2'
WPILIB = 'WPILib docs, Introduction to System Identification and Introduction to DC Motor Feedforward, docs.wpilib.org'
MOTOR = 'VEX V5 Smart Motor (11W), vexrobotics.com/276-4840.html; VEX Library, kb.vex.com'

PAGES = []


def page(*args, **kwargs):
    PAGES.append(Page(*args, **kwargs))


def test_of(key, size):
    return next(t for t in AT[key]['tests'] if t['sizeIn'] == size)


# ------------------------------------------------------------------ brain GUI

_gcrit = [('Change settings by touch, no re-download', 5), ('Readable at a glance in the pits', 4),
          ('Supported by the current PROS', 4), ('Can draw pictures (field map)', 3), ('Little work to build', 2)]
_gscores = [[2, 1, 5], [3, 2, 5], [1, 5, 5], [1, 1, 5], [5, 4, 2]]
page('Brainstorm and Pick Best Solution', 'A Screen for the Robot', [
    H('The problem'),
    P('Phase 4 of the plan is tools. Until now, changing any number (a PID gain, which autonomous to run) meant '
      'editing code and downloading it again, about a minute each time and only the programmer could do it. The V5 '
      'Brain has a 480 x 240 touch screen we were not using.'),
    H('Three approaches'),
    TABLE([
        ['', 'A. PROS text screen (LLEMU)', 'B. Controller screen only', 'C. Our own tabbed touch screen'],
        ['What it is', 'Eight lines of text and three buttons on the brain', 'Three lines of text on the controller, driven by its buttons', 'Pages with buttons and drawings, built on the LVGL graphics library that ships with PROS'],
        ['Good', 'Almost no code', 'Driver can use it from the field', 'Anything can be shown; touch buttons; a field map'],
        ['Bad', 'Marked as outdated by PROS, and it did not display for us on PROS 4', 'Three short lines; drops text sent faster than every 50 ms', 'Most work; LVGL has rules that are easy to break'],
    ], widths=[1.1, 2.2, 2.2, 2.4], size=10, bold_first_col=True),
    MATRIX(_gcrit, ['A. Text screen', 'B. Controller', 'C. Tabbed screen'], _gscores),
    BOX('**Decision (Cody Paschal):** build our own tabbed screen (C). The deciding criterion is the top one: the whole '
        'reason for a screen is to let anyone on the team tune and select without a laptop. We keep the controller '
        'screen (B) as well, for the three lines the driver needs during a match.', fill='FFF2CC'),
    H('Why tabs'),
    P('Our screens in past seasons used tabs and they were the easiest for the whole team to use, so we kept the idea: '
      'a row of tabs along the top, one page per job.'),
], PROS + ' (LLEMU, LVGL); ' + LVGL + '; ' + EZ + ' (auton selector and PID tuner); 96671H 2025-26 notebook.', O)

page('Programming', 'Building the Brain Screen', [
    SIDE('gui_sketch.png', 4.1, [
        H('Layout'),
        P('A header with the team name, a row of tabs, and one page under them. The sketch shows the PID tab: the '
          'controller list is a fixed column on the right.', 11.5),
    ], caption='Layout sketch of the PID tab (480 x 240 pixels).'),
    TABLE([
        ['Tab', 'What it shows', 'Why we wanted it'],
        ['Home', 'Battery, match status, heading, SD card state', 'Catch a dead battery or missing card in the pits'],
        ['Auton', 'List of routines; tap one to select it', 'Choose autonomous without a laptop'],
        ['Diag', 'Every motor and sensor: plugged in or not', 'A loose cable shows up before the match'],
        ['PID', 'Gains with + and - buttons, Run Test, Auto-Tune', 'Tune on the robot, see the effect at once'],
        ['Odom', 'Live x, y, heading and a dot on a field map', 'Check position tracking by pushing the robot'],
    ], widths=[0.9, 3.6, 3.4]),
    H('How a page is written'),
    P('Every tab is a "Page" with three parts. Adding a tab of our own uses exactly the same three parts as the '
      'built-in ones, so the screen can grow with the robot.'),
    CODE(r'''
class OdometryPage : public Page {
    const char* title() override { return "Odom"; }   // the text on the tab
    void build(lv_obj_t* parent) override;            // create the labels and buttons, once
    void update() override;                           // refresh the numbers, many times a second
};
gui.addPage(odometryPage);                            // built-in and custom pages: added the same way
'''),
    H('The rule that shaped the design'),
    P('LVGL is not "thread-safe": if two parts of the program touch the screen at the same instant, it can crash or '
      'draw garbage. A tuning run takes several seconds and cannot freeze the screen, so it has to run in the '
      'background. Our rule: **background work only ever writes plain numbers; only update() turns numbers into text '
      'on the screen.** An early draft of the tuner broke this rule by setting a label directly from the background '
      'task. It was caught by re-reading the rule against the code before it was ever downloaded.'),
    BOX('The screen is opt-in: a team using SapphireLib that wants its own screen, or none, simply never creates it.'),
], LVGL + ' (tabview, timers, threading); ' + PROS + ' (LVGL in PROS 4); ' + EZ + '; SapphireLib src/sapphirelib/gui/.', O)

page('Test', 'Brain Screen on the Real Brain', [
    H('Procedure'),
    P('The screen was the first part of SapphireLib tested on real hardware. Each version was downloaded to the brain '
      'on the test chassis and used with a finger the way it would be in the pits: switch tabs, press every button, '
      'push the robot around while watching the Odom tab. We recorded what was hard to use and changed it.'),
    TABLE([
        ['What we observed on the brain', 'Measured or noted', 'Change made'],
        ['Tabs were hard to hit with a finger', 'Tab bar was 28 pixels tall', 'Raised to 31, still missed; raised to 43 pixels, which works'],
        ['Scrolling a list by dragging was "incredibly hard"', 'The touch panel does not report a drag smoothly', 'No page needs dragging: the PID selector became a fixed column of buttons'],
        ['The whole screen felt laggy', 'All five pages refreshed every tick, even hidden ones', 'Only the visible tab refreshes'],
        ['Still redrawing when nothing changed', 'Setting a label to the same text still forces a redraw', 'Compare first; only write a label when its text changed'],
        ['Heading line on the map could be drawn wrong', 'Code review: the line was reading memory that had already been reused', 'The line\'s points are now kept by the page for as long as it exists'],
        ['Bottom of the field map was cut off', 'Map was 15 pixels taller than the page area', 'Map shrunk to 150 pixels; scrolling turned off'],
    ], widths=[2.6, 2.6, 2.7], size=10),
    H('Result'),
    P('After these changes the screen is confirmed working on the real brain: tabs switch on the first touch and the '
      'Odom page follows the robot as it is pushed.'),
    H('What this taught us'),
    B([
        '**Qualitative data counts.** "Hard to hit" is not a number, but it was the most useful finding on this page, '
        'and it only appears when a person uses the real thing.',
        '**Desktop tests and the robot find different bugs.** Unit tests checked the map math and passed. None of the '
        'six problems above could have been found on a laptop.',
        '**One of the six was a real memory bug,** not a looks problem. It worked "most of the time", which is the '
        'most dangerous kind.',
    ]),
    BOX('**Next:** the PID tab has + and - buttons, but tuning by hand is slow and depends on the tuner\'s experience. '
        'The next pages are our three attempts at making the robot tune itself.'),
], LVGL + ' (label, line, scrolling); ' + PROS + '; SapphireLib docs/ROADMAP.md (Phase 4 notes) and src/sapphirelib/gui/.', O)

# ------------------------------------------------------------------ autotuners

page('CAD/Design and Prototype', 'Autotuner Prototype 1: Twiddle', [
    H('Why an autotuner'),
    P('A PID needs three numbers (gains). Finding them by hand means: change one, run the robot, watch, repeat, for '
      'every controller on the robot, again after every rebuild. Criterion 4 on page G-1 says the software should do '
      'this itself. We prototyped three ways.'),
    SIDE('twiddle.png', 3.8, [
        H('How Twiddle works'),
        P('Twiddle is "try it and see", automated. It is taught in Sebastian Thrun\'s robotics course.', 11.5),
        B([
            'Nudge one gain up. Run a test drive and score it.',
            'Better? Keep it and take a bigger nudge next time.',
            'Worse? Try nudging down instead. Still worse? Put it back and nudge smaller.',
            'Move to the next gain. Repeat until the nudges are tiny.',
        ], 11.5),
    ]),
    H('Test (simulator)'),
    P('**Procedure:** simulated forward axis of our drive. Each trial is a 24 in drive, scored by total error over 3 s '
      'plus a penalty for overshoot. Start from kP 0.3, kI 0, kD 0. Stop when the nudges add up to under 0.02 or after '
      '200 trials.'),
    TABLE([
        ['Measured', 'Result'],
        ['Test drives used', '%d (stopped at the limit)' % AT['twiddle']['runs']],
        ['Robot time that would take', 'About %d minutes of driving, plus resetting the robot each time' % round(AT['twiddle']['robotSeconds'] / 60)],
        ['Trials that overshot badly or never settled', '%d of %d' % (AT['twiddle']['unstableRuns'], AT['twiddle']['runs'])],
        ['Gains it ended on', 'kP %.2f, kI %.2f, kD %.3f' % (AT['twiddle']['gains']['kP'], AT['twiddle']['gains']['kI'], AT['twiddle']['gains']['kD'])],
        ['24 in drive with those gains', 'Settles in %.1f s, overshoot %.1f in' % (test_of('twiddle', 24)['settleS'], test_of('twiddle', 24)['overshootIn'])],
        ['6 in drive with those gains', 'Stops %.2f in short and never gets closer' % test_of('twiddle', 6)['finalErrorIn']],
    ], widths=[3.4, 4.5], fills={(3, 1): RED, (6, 1): RED}),
    H('What we learned'),
    B([
        'It works, slowly. But the red points on the chart are trials where the robot overshot or shook. On a real '
        'robot those are runs into a wall.',
        'It only learns the one test it is scored on. Gains tuned on a 24 in drive were too gentle to finish a 6 in drive.',
        'It tells us nothing about the robot. Tuning the turn controller means starting again from zero.',
    ]),
    BOX('**Decision:** do not ship Twiddle. Keep its idea of scoring a test drive, and look for a method that measures '
        'the robot instead of searching blindly. Decided by Cody Paschal.', fill='FFF2CC'),
], 'S. Thrun, "Artificial Intelligence for Robotics" (Udacity), lesson on PID control (Twiddle); ' + EZ + ' (manual PID tuning guide); SapphireLib simulator, run by notebook/build/experiments.js.', O)

page('CAD/Design and Prototype', 'Autotuner Prototype 2: Relay (Ziegler-Nichols)', [
    H('The idea'),
    P('Astrom and Hagglund (1984) showed a way to tune from a single experiment. Instead of a PID, use a "relay": full '
      'push one way until the robot crosses the target, then full push the other way. The robot settles into a steady '
      'wobble. How fast it wobbles and how far tell you how the system behaves, and the Ziegler-Nichols rules (1942) '
      'turn those two numbers into gains.'),
    SIDE('relay.png', 3.8, [
        H('Our prototype'),
        B([
            'Switch the drive between +6 V and -6 V each time it crosses the start line.',
            'Wait for the wobble to become steady.',
            'Measure its period (Tu) and its size.',
            'Ku = 4 x 6 V / (pi x size). Then kP = 0.6 Ku, kI = 1.2 Ku / Tu, kD = 0.075 Ku Tu.',
        ], 11.5),
    ]),
    H('Test (simulator)'),
    TABLE([
        ['Measured', 'Result'],
        ['Test drives used', '1 wobble run, about %d s' % AT['relay']['robotSeconds']],
        ['Wobble', 'Period %.2f s, %.2f in each way' % (AT['relay']['TuS'], AT['relay']['amplitudeIn'])],
        ['Gains', 'kP %.2f, kI %.1f, kD %.3f' % (AT['relay']['gains']['kP'], AT['relay']['gains']['kI'], AT['relay']['gains']['kD'])],
        ['24 in drive with those gains', 'Settles in %.2f s, overshoot %.1f in' % (test_of('relay', 24)['settleS'], test_of('relay', 24)['overshootIn'])],
        ['6 in drive with those gains', 'Settles in %.2f s, overshoot %.1f in' % (test_of('relay', 6)['settleS'], test_of('relay', 6)['overshootIn'])],
    ], widths=[3.4, 4.5], fills={(4, 1): YELLOW, (5, 1): RED}),
    H('What we learned'),
    B([
        '**Big improvement in effort:** one 8 s run replaces hundreds of trials, and the wobble is small and safe.',
        '**But it overshoots at every distance.** On a 6 in drive the overshoot is %.1f in, a third of the move. '
        'Ziegler-Nichols was designed for chemical plants, where a quick response matters more than overshoot. A robot '
        'stopping at a goal needs the opposite.' % test_of('relay', 6)['overshootIn'],
        '**It measures one speed of wobble only** and assumes a smooth wave. A drivetrain has friction and gear slop, '
        'so the real wave is not smooth.',
        '**One set of gains per mechanism.** The turn controller in autonomous and the heading hold in driver control '
        'need different behaviour from the same motors, and the relay can only give one answer.',
    ]),
    BOX('**Reflection:** we used this version for over three weeks and every result still needed fine tuning by hand. '
        'That is the evidence that sent us back to research. The useful lesson was that one short experiment can replace '
        'a search. The missing piece was getting more than two numbers out of it.'),
], 'K. Astrom and T. Hagglund, "Automatic tuning of simple regulators with specifications on phase and amplitude margins," Automatica 20(5), 1984; J. Ziegler and N. Nichols, "Optimum settings for automatic controllers," Trans. ASME 64, 1942; ' + ASTROM + '.', O)

page('CAD/Design and Prototype', 'Autotuner Prototype 3: Measure a Model', [
    H('The idea'),
    P('FRC teams tune with "system identification" (WPILib\'s SysId): first measure a simple model of the mechanism, '
      'then calculate the gains from the model. The model answers "how many volts does it take?" with three numbers:'),
    SIDE('ff_model.png', 3.5, [
        BOX('**Volts = kS + kV x speed + kA x acceleration**', fill='E3F2FD', size=11.5),
        B([
            '**kS:** volts just to overcome friction.',
            '**kV:** extra volts for each in/s of speed.',
            '**kA:** extra volts for each in/s^2 of speeding up.',
        ], 11.5),
        P('Once these are known, we know how the robot will respond to any voltage, and can choose gains by '
          'calculation instead of by trial.', 11.5),
    ]),
    IMGB('flow_autotune.png', caption='One press of Auto-Tune runs these five steps for each axis: forward, strafe and turn.'),
    H('Two problems we had to solve'),
    B([
        '**Noise.** The obvious way to find acceleration is to take the change in speed, which is the change in '
        'position, twice over. Each step multiplies sensor noise, and noisy acceleration makes kA come out too small. '
        'We instead fit the exact equation of how speed changes from one moment to a later one, which never needs '
        'acceleration.',
        '**Delay.** A command takes time to reach the wheels (about 30 ms in our model). Fitting delayed data as if it '
        'were instant makes friction look bigger and kA smaller. We measure the delay by comparing how long a step '
        'takes to reach half speed against what a delay-free model predicts, then fit again with the commands shifted.',
    ]),
    H('Test of the measuring step (desktop)'),
    P('We simulated an axis with known kS, kV, kA and delay, added sensor noise, and asked the code to recover the '
      'numbers. It does, to within the test\'s tolerance, and refuses to give an answer when the data is bad (a '
      'reversed sensor, too little travel), leaving the old gains in place.'),
    BOX('**What changed from prototype 2:** the same idea of one short experiment, but it now yields a full '
        'description of the axis instead of two numbers. One measurement of the turn axis can design the autonomous '
        'turn controller and the driver heading hold, each for its own job.'),
], WPILIB + '; ' + ASTROM + ', ch. 2 (System Modeling) and ch. 7 (Output Feedback); SapphireLib tests/tuning/characterization_math_test.cpp.', O)

_tcrit = [('Good gains at any distance', 5), ('Safe while tuning (no wild runs)', 4),
          ('One measurement tunes every controller', 4), ('Little robot time', 3), ('Simple to understand', 2)]
_tscores = [[2, 3, 5], [1, 4, 4], [1, 1, 5], [1, 5, 4], [5, 3, 2]]
page('Test', 'Comparing the Three Autotuners', [
    H('Procedure (repeatable)'),
    B([
        'To compare fairly, all three methods were run again on the **same** simulated forward axis (1.0 V friction, '
        '0.20 V per in/s, 0.045 V per in/s^2, 30 ms delay, 12 V limit, 10 ms loop).',
        'Each method produced its gains with no hand adjustment.',
        'Each set of gains then drove 6, 24 and 48 in. Measured: time until it stays within 0.5 in, and overshoot.',
    ]),
    IMGB('autotuner_compare.png'),
    TABLE([['Tuner', 'Test drives needed', 'Gains (kP / kI / kD)', '6 in drive', '24 in drive', '48 in drive']] + [
        [label, runs,
         '%.2f / %.1f / %.3f' % (AT[k]['gains']['kP'], AT[k]['gains']['kI'], AT[k]['gains']['kD']),
         *['%s, +%.1f in' % (('%.2f s' % test_of(k, s)['settleS']) if test_of(k, s)['finalErrorIn'] <= 0.5 else 'never', test_of(k, s)['overshootIn']) for s in (6, 24, 48)]]
        for k, label, runs in (('twiddle', 'Twiddle', str(AT['twiddle']['runs'])), ('relay', 'Relay', '1'), ('model', 'Model', '4 short runs'))
    ], widths=[0.9, 1.2, 1.9, 1.3, 1.3, 1.3], size=9.5,
        fills={(1, 3): RED, (2, 3): RED, (2, 4): YELLOW, (2, 5): YELLOW, (3, 3): GREEN, (3, 4): GREEN, (3, 5): GREEN, (1, 1): RED}),
    P('Each drive cell shows settle time, then overshoot. The model-based gains are the only set with no overshoot at '
      'all three distances.', 11),
    MATRIX(_tcrit, ['Twiddle', 'Relay', 'Model'], _tscores),
    BOX('**Decision (Cody Paschal): model-based Auto-Tune replaces the relay tuner.** The evidence is the table: zero '
        'overshoot at all three distances from four short runs. Its one weak score, being harder to understand, is why '
        'the next page explains the gain calculation step by step.', fill='FFF2CC'),
    P('**Reflection:** three prototypes, each built on what the last one taught: Twiddle showed that scoring a drive '
      'works but searching is unsafe; the relay showed one experiment is enough but two numbers are not; the model '
      'keeps the single experiment and gets the full picture.', 11.5),
], WPILIB + '; Astrom and Hagglund 1984; Ziegler and Nichols 1942; S. Thrun (Twiddle); SapphireLib simulator, run by notebook/build/experiments.js.', O)

page('Programming', 'Auto-Tune: From Model to Gains', [
    P('With kS, kV and kA measured, choosing gains becomes a calculation called **pole placement**. We decide how the '
      'robot should behave, and solve for the gains that make it so. We describe "how it should behave" with two '
      'numbers a person can picture:'),
    TABLE([
        ['We choose', 'Meaning', 'Our drive controller'],
        ['Settle time', 'How long until the robot is at the target and staying there', '0.6 s'],
        ['Damping', '1.0 = as fast as possible with no overshoot; lower = faster but overshoots', '1.0'],
        ['Minimum phase margin', 'A safety margin against delay making it shake (see below)', '50 degrees'],
    ], widths=[1.8, 4.3, 1.8]),
    CODE(r'''
// omega = how "fast" the response is. A faster settle time needs a bigger omega.
// zeta  = damping (1.0 = no overshoot)
PIDGains placePoles(const MotorFeedforward& model, double omega, double zeta) {
    return PIDGains{
        .kP = model.kA * omega * omega,          // heavier robot (bigger kA) -> more push
        .kI = 0.0,                               // not needed: kS already handles friction
        // the brake, minus the braking the motors already do themselves (kV)
        .kD = std::fmax(0.0, 2.0 * zeta * omega * model.kA - model.kV),
    };
}

// delay is what the textbook formula ignores. If the measured delay leaves too little
// safety margin, slow the design down until it is safe, and report that we did
double omega = unitSettle / spec.settleTimeS;
if (phaseMarginDeg(model, placePoles(model, omega, zeta), delayS) < spec.minPhaseMarginDeg) {
    omega = slowestOmegaThatMeetsTheMargin(model, zeta, delayS, spec);
}
'''),
    H('Why there is no I term'),
    P('The integral exists to remove the small error friction leaves. Our model already knows the friction (kS), so '
      'the controller can add it directly. That removes integral windup (page [[windup]]) at its source.'),
    H('Phase margin in plain words'),
    P('Steering a boat with a slow rudder: react too strongly and by the time the boat responds you have '
      'over-corrected, and it weaves. Phase margin measures how close a controller is to weaving because of delay. '
      'For the simulated drive the design came out at %.0f degrees, above our 50 degree minimum.' % AT['model']['design']['phaseMarginDeg']),
    H('What Auto-Tune does on our robot'),
    TABLE([
        ['Axis measured once', 'Controllers designed from it'],
        ['Forward', 'Drive, path tracking (forward)'],
        ['Strafe', 'Path tracking (sideways)'],
        ['Turn', 'Autonomous turn, driver heading hold, path tracking (turn): three different settle times'],
    ], widths=[2.3, 5.6]),
    BOX('**Status:** the gains our robot runs today came from this Auto-Tune. This is also where the per-second gain '
        'bug was found: the formula\'s gains were right, but the old per-loop PID applied them 100 times too strongly '
        'in I and 100 times too weakly in D.'),
], ASTROM + ', ch. 7 (pole placement) and ch. 10 (phase margin); ' + WPILIB + '; SapphireLib src/sapphirelib/tuning/gain_design.cpp.', O, key='autotune_code')

page('Programming', 'Auto-Tune for the Lift', [
    H('What is different about a lift'),
    P('The drive model assumes that with no voltage, the robot sits still. A lift does not: gravity pulls it down all '
      'the time. So the lift\'s model has a fourth number, **kG**, the volts needed just to hold the lift up.'),
    BOX('**Volts = kS x (direction) + kG + kV x speed + kA x acceleration**', fill='E3F2FD'),
    H('How we separate gravity from friction'),
    P('Both kS and kG are "volts before anything moves", so one experiment cannot tell them apart. The trick: friction '
      'always fights the motion, so it flips sign when the lift changes direction. Gravity always pulls down. By '
      'running the lift **up and down**, the difference between the two directions reveals which is which.'),
    TABLE([
        ['Going up', 'Going down', 'So'],
        ['Motors fight friction AND gravity', 'Gravity helps; motors fight friction only', 'Half the difference in volts is gravity; the average is friction'],
    ], widths=[2.5, 2.7, 2.7]),
    H('Two kinds of mechanism'),
    TABLE([
        ['Type', 'Gravity term', 'Example'],
        ['Elevator (moves straight up)', 'Constant: kG', 'Our cascade lift'],
        ['Arm (swings)', 'kG x cos(angle): most when level, zero when straight up', 'A four-bar or wrist'],
    ], widths=[2.7, 3.3, 1.9]),
    H('Safety built into the run'),
    B([
        'The lift moves only between two limits we set, and is braked between each part of the test.',
        'A Stop button on the screen ends a run at once. So does disabling the robot.',
        'New gains are applied only if the whole run succeeded. A failed run leaves everything as it was.',
        'Tuning the lift never moves the drive, and the other way round.',
    ]),
    H('Result'),
    P('The measured kG goes straight into the lift as "gravity feedforward": the lift always gets its holding voltage, '
      'and the PID only has to handle the movement. The lift holds its height without an I term slowly winding up.'),
    CODE(r'''
// the lift's control, each loop: PID for the movement + a constant to cancel gravity
volts = pid.update(targetHeight, currentHeight, dt) + gravityVolts;
'''),
    BOX('**Reflection:** writing the lift version was quick because it reuses the drive version\'s experiment and fit '
        'with one more term. That is the pay-off from prototype 3: a model can be extended, a search or a wobble cannot.'),
], WPILIB + ' (elevator and arm feedforward); ' + ASTROM + '; SapphireLib src/sapphirelib/tuning/ and tests/tuning/characterization_runner_test.cpp.', O)

page('Monthly Gantt Charts', 'SapphireLib Progress Check', [
    IMGB('gantt_mid.png'),
    H('Original goal'),
    P('SapphireLib driving the competition robot in autonomous by mid-September.'),
    H('Accomplished'),
    B([
        'Research and the decision to write our own library.',
        'Phase 1: PID controller, tank and holonomic drives, field-centric control.',
        'Phase 2: odometry for all four sensor setups, with offset calibration.',
        'Phase 3: moveToPoint, moveToPose, followPath, each returning a result.',
        'Phase 4 (part): the brain screen, working on the real brain, and three autotuner prototypes.',
    ]),
    H('Partially complete'),
    B(['Automatic tuning: the model-based method is chosen and tested in simulation; the lift version is being added.']),
    H('Pending'),
    B(['Driver macros, SD card logging, and moving everything to the competition robot.']),
    H('Analysis and reasoning'),
    P('Each solid bar starts later and runs longer than its hatched plan. Two causes. First, the motion bugs on page '
      '[[buglog]] took time we had not planned for. Second, the autotuner took three prototypes instead of one: the '
      'relay tuner alone was in use for over three weeks before we accepted it was not good enough. We chose to spend '
      'that time because tuning is repeated after every rebuild all season, so a tuner that needs hand-fixing costs '
      'more in total than building a better one once.'),
    H('Future plans'),
    P('The plan had the competition robot port starting in late August. That now moves back. The asterisk drive '
      'changes, macros and logging come next because the competition robot cannot be driven without them. The "extra '
      'goal" from page [[gantt_plan]], correcting drift with distance sensors, is kept, because the bump test on page '
      '[[drift]] shows odometry alone will not be accurate enough.'),
], '96671H notebook, page A-4 (How to Read Gantt Charts); SapphireLib docs/ROADMAP.md.', O, key='gantt_mid')

# ------------------------------------------------------------------ asterisk drive, macros, telemetry

page('Programming', 'Asterisk Drive: Using All Six Motors', [
    H('What testing showed'),
    P('We checked what voltage the mixer sends each of the six motors for each kind of motion. For a turn in place, '
      'the two center motors were sent 0 V. Four motors were turning the robot while two were being dragged.'),
    IMGB('asterisk_turn.png'),
    H('Why it happened'),
    P('The first mixer gave the center wheels only the "forward" part of the command. A pure turn has zero forward '
      'part, so they got zero. That looked reasonable when written: a straight wheel cannot help a robot slide '
      'sideways, so we had assumed it could not help a turn either.'),
    H('Why that assumption was wrong'),
    P('In a turn about the center of the robot, a wheel on the left side travels straight backward and a wheel on the '
      'right side straight forward. That is exactly the direction a straight center wheel rolls. They are the '
      'best-placed wheels on the robot for turning.'),
    CODE(r'''
// before: center wheels got the forward part only
middleLeft  = throttleVolts;
middleRight = throttleVolts;

// after: they also get the turn part, left and right opposite
middleLeft  = throttleVolts + asterisk.turnContribution * turnVolts;
middleRight = throttleVolts - asterisk.turnContribution * turnVolts;
'''),
    TABLE([
        ['', 'Before', 'After'],
        ['Motors working in a turn', '4 of 6', '6 of 6'],
        ['Center wheels during a turn', 'Dragged sideways across the tile', 'Driven in the direction they roll'],
        ['Straight-line heading correction', 'Corners only', 'All six'],
    ], widths=[2.9, 2.5, 2.5], fills={(1, 1): RED, (1, 2): GREEN}),
    P('turnContribution is a setting (default 1.0). Setting it to 0 gives the old behaviour, so the change can be '
      'switched off to compare.'),
    BOX('**Traceable change:** the check of the mixer\'s output (center motors at 0 V in turns) led to the mixer '
        'change above. Decided by Cody Paschal.', fill='FFF2CC'),
], GM0 + ' (mecanum kinematics); 96671H notebook, Section D (asterisk drive testing); SapphireLib src/sapphirelib/chassis/holonomic_drivetrain.cpp.', G)

page('Programming', 'Heat: Keeping a Strafe Straight', [
    SIDE('thermal.png', 3.7, [
        H('The problem'),
        P('A V5 motor protects itself as it heats up by cutting its own power: to about half at 55 C and to nothing '
          'at 70 C. It does not tell the program it has done so.', 11.5),
        P('On a tank drive that just means "slower". On a mecanum drive it is worse. A strafe works because the four '
          'corners cancel each other\'s forward and backward push exactly. If **one** corner is weaker, they no longer '
          'cancel, and the robot creeps forward or backward and twists while it strafes.', 11.5),
    ]),
    H('Our solution: the center wheels cancel the drift'),
    P('The two straight center wheels face exactly the direction of that unwanted creep. So:'),
    B([
        'Twice a second, read each corner motor\'s temperature and estimate what fraction of its power it still has '
        '(the curve above).',
        'Every loop, multiply each corner\'s commanded volts by its shortfall. That is the push that is missing.',
        'Add up the missing push in the forward direction and the turning direction, and give that to the center '
        'wheels, which can supply both.',
    ]),
    CODE(r'''
// each corner's missing push = what we asked for x the fraction heat has taken away
missing[i] = commandedVolts[i] * (1.0 - thermalPowerFraction(tempC[i]));

// the center wheels can push forward/back and can turn. Project the missing push onto those
forwardFix = (missing[FL] + missing[FR] + missing[BL] + missing[BR]) / 4.0;
turnFix    = (missing[FL] - missing[FR] + missing[BL] - missing[BR]) / 4.0;
'''),
    H('Design decisions'),
    TABLE([
        ['Choice', 'Why'],
        ['Per corner, never an average', 'Two hot corners on the same diagonal double the drift; one on each diagonal cancel. An average cannot tell these apart. A desktop test checks exactly this case.'],
        ['Act before the drift happens (feedforward)', 'A separate loop on the tracking wheel catches drift after it starts. This one prevents the part we can predict.'],
        ['Fade out as the center motors heat up', 'Otherwise fixing a 4-motor overheat creates a 6-motor overheat.'],
        ['A cap on the correction', 'A bad temperature reading can never command a large voltage.'],
    ], widths=[2.8, 5.1]),
    BOX('**Limit we accept:** nothing mounted straight can push sideways. A strafe on hot corners holds its line but '
        'is still slower. **Status:** the math is tested on the desktop; the simulator does not model heat, so this is '
        'the least-tested feature in this section and is on our list for a long practice match.'),
], MOTOR + '; ' + GM0 + '; SapphireLib src/sapphirelib/chassis/thermal_math.cpp and tests/chassis/thermal_math_test.cpp.', G)

page('Programming', 'Heading Hold and Drift Correction', [
    H('Heading hold: the turn stick steers a direction'),
    P('In the first driver code, the turn stick set how fast the robot spins. When a robot hit ours, it stayed at '
      'whatever angle it was knocked to and the driver had to notice and fix it. We changed what the stick means: it '
      'now moves a **target heading**, and the turn PID holds the robot on that heading all the time.'),
    TABLE([
        ['', 'Stick = turn speed (before)', 'Stick = target heading (after)'],
        ['Let go of the stick', 'Robot coasts on a little', 'Robot is pointed somewhere definite'],
        ['Hit by another robot', 'Stays knocked off line', 'Turns itself back'],
        ['One hot motor in a strafe', 'Robot twists as it slides', 'Twist is corrected'],
    ], widths=[2.3, 2.8, 2.8], fills={(2, 1): RED, (2, 2): GREEN}),
    H('Three details that made it usable'),
    TABLE([
        ['Problem found', 'Cause', 'Fix'],
        ['Robot slowly rotated with no hands on the controller', 'A resting stick reads a count or two, and a held heading adds that up all match', 'Deadband on the stick'],
        ['Robot kept turning after the stick was released', 'The target ran far ahead of the robot, which then had to catch up', 'Cap on how far the target may lead the robot'],
        ['Robot spun at the start of driver control', 'The held heading was from before autonomous', 'After any gap over 0.25 s, re-adopt the current heading and reset the PID'],
    ], widths=[2.6, 3.1, 2.2], size=10),
    H('Drift correction: a bug in our own fix'),
    P('The center wheels also correct forward creep during a strafe, measured by the vertical tracking wheel. The '
      'first version treated **any** forward motion during a strafe as drift. Testing showed it fighting the driver:'),
    TABLE([
        ['What the robot was doing', 'What the first version did', 'What it should do'],
        ['Driving a diagonal on purpose', 'Fought the forward part', 'Nothing: that motion was asked for'],
        ['Turning while strafing', 'Read the tracking wheel\'s arc (0.06 in per degree) as drift', 'Remove the arc first'],
    ], widths=[2.4, 3.0, 2.5], size=10),
    CODE(r'''
// drift = what the wheel rolled - what turning explains - what the driver asked for
driftIn = verticalWheelDeltaIn - verticalOffsetIn * rotationRad - commandedForwardIn;
'''),
    BOX('**Reflection:** the wheel offset that odometry subtracts (page [[odomcode]]) mattered here too. The same '
        'physical fact, "an off-center wheel rolls when the robot turns", had to be handled in two places. We now ask '
        'of every new feature that reads a tracking wheel: is the robot turning?'),
], GM0 + ' (field-centric control); ' + ASTROM + ', ch. 10; SapphireLib src/sapphirelib/control/heading_hold.cpp, chassis/drift_math.cpp and their tests.', G)

page('Programming', 'Driver Macros', [
    H('The problem'),
    P('This game needs a lift, a claw, an intake and several pistons to work in sequence, but each must still be '
      'controllable on its own, and the controller has 12 buttons. Some buttons must mean different things at '
      'different times: our outtake button is also the score button, depending on whether the claw is deployed.'),
    IMGB('flow_macro.png', caption='The macro loop. Every output is worked out fresh from the robot\'s state each time round.'),
    H('Approaches we considered'),
    TABLE([
        ['Approach', 'How it works', 'Why we did or did not use it'],
        ['Command framework (FRC style)', 'Each action is an object; a scheduler decides what runs', 'Powerful, but adds rules to learn and hides the order things happen in'],
        ['One task per mechanism', 'Lift, claw and intake each run their own loop', 'They must talk to each other constantly; timing bugs'],
        ['One loop with a priority list (chosen)', 'A single function; the first rule that applies wins', 'The robot\'s priorities are readable top to bottom in one place'],
    ], widths=[2.2, 2.8, 2.9], size=10, fills={(3, 0): GREEN}),
    CODE(r'''
master.update();                                   // read every button once, at the top of the loop
if (scoring.running()) {                           // 1. a scoring sequence in progress wins
    scoring.update(now);
} else if (master.pressed(Button::a) && lift.level() > 0) {
    scoring.start(now);                            // 2. A = "dip, release, stow" on its own
} else {
    if (master.pressed(Button::l1)) lift.up();     // 3. otherwise the buttons mean the usual things
    if (master.pressed(Button::r1)) claw.toggle(now);
}
'''),
    H('Building blocks the library provides'),
    TABLE([
        ['Block', 'Replaces this hand-written code'],
        ['Controller', 'Detecting a new press, a hold, a long press, two buttons together'],
        ['PositionMechanism', 'A lift or arm: PID + gravity, rest at the bottom, stop if the sensor unplugs'],
        ['Piston', 'A pneumatic that knows how long it has been open ("wait 400 ms after deploying")'],
        ['Roller', 'An intake that notices a jam and reverses briefly'],
        ['Sequence', 'Timed steps that do not freeze driver control while they run'],
    ], widths=[2.1, 5.8]),
    BOX('**A bug this fixed:** PROS\'s own "new press" check only updates when a button is read. Our old two-button '
        'shortcut (B + DOWN) only worked if the buttons were pressed in one order. Reading every button once per loop '
        'and comparing with the last loop removes the problem. Decided by Cody Paschal.', fill='FFF2CC'),
], PROS + ' (controller API); WPILib command-based programming, docs.wpilib.org; ' + EZ + '; SapphireLib docs/MACROS.md.', G)

page('Test', 'Proving the Macro Rewrite Changed Nothing', [
    H('The risk'),
    P('The robot already had hand-written lift and scoring code that worked. Replacing it with the new building blocks '
      'could quietly change how the robot behaves, and the drivers had practised on the old behaviour.'),
    H('Procedure: old and new side by side'),
    P('For each piece, a copy of the **old** code was kept inside a desktop test. The test feeds the same inputs to the '
      'old and new code for a very large number of loops and fails if their outputs ever differ. The inputs are random '
      'but come from a fixed seed, so the test is repeatable.'),
    TABLE([
        ['Piece', 'Compared against', 'How much', 'Result'],
        ['Lift control', 'The old hand-written lift loop', '1.2 million simulated loops, including sensor dropouts', 'Identical, bit for bit'],
        ['Scoring sequence', 'The old phase machine', '10,000 random sequences of driver presses', 'Identical, step for step'],
        ['Button presses', 'A copy of PROS\'s own press logic', '400,000 random loops', 'Identical'],
        ['Controller screen', 'The old screen code', 'Every write compared', 'Identical'],
        ['Motion exit logic', 'The old loop inside each motion', 'Old and new drivetrain code replayed in a simulator harness', 'Every motor command unchanged'],
    ], widths=[1.6, 2.3, 2.6, 1.4], fills={(r, 3): GREEN for r in range(1, 6)}),
    H('Why "bit for bit"'),
    P('Close is not good enough here. If the new lift code gave 6.001 V where the old gave 6.000 V, the lift would '
      'still work, but we would no longer know whether a later problem came from the rewrite. Requiring an exact match '
      'means any future difference in behaviour is something we chose.'),
    H('What the new blocks add on top of matching'),
    B([
        'The B + DOWN shortcut now works whichever button is pressed first (the old order bug, previous page).',
        'A lift at the bottom can rest at 0 V on its hard stop instead of pushing into it for the whole match.',
        'A sequence changes at most one step per loop, so every step runs at least once and none can be skipped.',
        'If the lift\'s sensor stops answering, the lift brakes instead of driving on a bad reading.',
    ]),
    BOX('**Reflection:** this is the software version of test-fitting a new part before removing the old one. It took '
        'longer than the rewrite itself. **Limit:** these are desktop tests. They prove the logic matches, not that '
        'the real lift feels the same, which the drivers check in practice.'),
], PROS + '; SapphireLib tests/mechanism/position_control_test.cpp, tests/util/sequence_test.cpp, tests/input/ and tests/motion/exit_tracker_test.cpp.', G)

page('Identify the Problem', 'Recording What Happened: SD Card Telemetry', [
    H('The problem'),
    P('When something fails in a match (a motion stops short, the lift fades in the last 30 seconds) we could only '
      'guess at the cause. Criterion 5 on page G-1 asks the software to explain itself. We need a recording of what '
      'the robot was doing, from inside the robot.'),
    H('Criteria and constraints'),
    TABLE([
        ['Requirement', 'Why'],
        ['Logging must **never** slow down the code it records', 'A logger that makes the PID loop late causes the problems it is meant to find'],
        ['Record every PID step, the pose, and every motor', 'These are the three places problems start'],
        ['Survive the card being pulled or filling up', 'Cards get bumped in matches'],
        ['Readable without special software', 'Anyone on the team should be able to open a log'],
    ], widths=[4.0, 3.9]),
    H('What PROS and the V5 limit, and our answer to each'),
    TABLE([
        ['Limit we found', 'Design answer'],
        ['Every write to the SD card is slow (a full sync)', 'Collect rows in memory; write one big chunk every 250 ms'],
        ['PROS deletes tasks when the match mode changes', 'No locks anywhere a control loop runs, so a deleted task cannot leave one locked'],
        ['The brain has no clock or date', 'Files are numbered: SL000001, SL000002, ...'],
        ['PROS cannot create folders', 'Use the "sl" folder if it exists, otherwise the card\'s top level'],
        ['Two tasks using the card at once can corrupt it', 'One lock for the card, held only by slow background tasks'],
    ], widths=[3.6, 4.3]),
    IMGB('flow_telemetry.png', caption='The fast side (left) never waits for the slow side (right).'),
    H('The key design choice: drop and count'),
    P('If the memory buffer is full, a new row is **dropped** rather than making the control loop wait, and the number '
      'dropped is written into the file. A gap we can see in the data is better than a robot that stutters.'),
    BOX('**Decision (Cody Paschal):** logging starts by itself whenever the robot is plugged into field control, and '
        'from a button on the Home tab otherwise. Every match is recorded without anyone remembering to press anything.',
        fill='FFF2CC'),
], PROS + ' (filesystem, tasks, competition control); ' + LEMLIB + ' (logging); SapphireLib docs/TELEMETRY_FORMAT.md and src/sapphirelib/telemetry/.', G)

page('Programming', 'Reading the Logs: the Analyzer and the Tuning Loop', [
    H('A log nobody reads is worth nothing'),
    P('A 2 minute match produces thousands of rows. We built a companion tool, the telemetry analyzer, that runs in a '
      'web browser with nothing to install: drop the SD card\'s files onto the page.'),
    TABLE([
        ['What the analyzer does', 'Example'],
        ['Lists what went wrong, most serious first, each with its time', '"Lift motor reached 55 C at 1:12 and lost half its power"'],
        ['Replays the match from above', 'The robot\'s path, its targets, and the lift\'s height beside it'],
        ['Charts any recorded value against time', 'Battery voltage against drive current'],
        ['Tunes from the log', 'Re-fits the Auto-Tune model from ordinary driving and proposes gains'],
    ], widths=[4.1, 3.8]),
    H('Findings the analyzer looks for'),
    B([
        '**Motors:** overheating (and which power step it reached), unplugging, stalling.',
        '**Mechanisms:** stuck short of a target at full power.',
        '**Motions:** timed out, and whether the robot was still pushing when they did.',
        '**Electrical:** battery sag, a controller dropping out, a sensor unplugged.',
        '**The logger itself:** rows dropped, SD faults.',
    ]),
    H('Closing the loop: TUNE.CFG'),
    P('Finding a better gain is only half of it. Getting it onto the robot used to mean editing code and '
      'rebuilding. Now the tools write a small text file, TUNE.CFG, to the SD card, and the robot reads it when it '
      'starts.'),
    IMGB('flow_tuning.png'),
    CODE(r'''
format=1
rev=8
note=PID refinement: Turn
pid.turn=0.442,0,0.0849        # kP,kI,kD
model.turn=0.6,0.045,0.00992   # kS,kV,kA
mcl.periodMs=33
mcl.filter.motionNoise.perInch=0.07
'''),
    B([
        'Every line is checked against the allowed names and ranges. **One bad line rejects the whole file** and the '
        'robot uses the values in its code, so a typo can never half-apply.',
        'The file\'s revision number is shown on the Home tab and stamped into every log, so we always know which '
        'settings a match was played with.',
    ]),
    BOX('**Reflection:** telemetry changed how we work more than any other feature. Every "Test" page after this one '
        'that mentions a log (for example SL000029) is quoting a file recorded by this system.'),
], PROS + '; WPILib DataLog and AdvantageScope (log replay tools in FRC), docs.wpilib.org; SapphireLib tools/analyzer/README.md and docs/TUNING.md.', G)
