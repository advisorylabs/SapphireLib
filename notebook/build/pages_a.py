"""Section G pages, part 1: the problem, research, the decision, PID and chassis, odometry and motions."""
import json
import os

from nb import B, BOX, CODE, H, IMGB, MATRIX, P, SIDE, TABLE, Page, GREEN, RED, YELLOW, GRAY, BLUE

HERE = os.path.dirname(os.path.abspath(__file__))


def data(name):
    with open(os.path.join(HERE, 'data', name + '.json'), encoding='utf-8') as f:
        return json.load(f)


WIND = data('pid_windup')
KICK = data('pid_kick')
PERIOD = data('pid_period')
LOOK = data('lookahead')

R, C, K, O, G, T = ('Research and decision', 'Core control: PID + chassis', 'Odometry + pose motions',
                    'Brain GUI + autotuners', 'Asterisk drive, macros, telemetry', 'Monte Carlo localization')

LEMLIB = 'LemLib, github.com/LemLib/LemLib and lemlib.readthedocs.io'
EZ = 'EZ-Template, ez-robotics.github.io/EZ-Template'
PROS = 'PROS docs, pros.cs.purdue.edu/v5'
SIGBOTS_ODOM = 'Purdue SIGBots Wiki, Odometry, wiki.purduesigbots.com/software/odometry'
SIGBOTS_PID = 'Purdue SIGBots Wiki, PID Controller, wiki.purduesigbots.com/software/control-algorithms/pid-controller'
PILONS = '5225A E-Bots PiLons, "Introduction to Position Tracking" (2018), thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf'
COULTER = 'R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01 (1992)'
ASTROM = 'K. Astrom and R. Murray, Feedback Systems (Princeton, 2008), fbswiki.org'
GM0 = 'Game Manual 0, Mecanum Drive, gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html'

PAGES = []


def page(*args, **kwargs):
    PAGES.append(Page(*args, **kwargs))


# ------------------------------------------------------------------ research and decision

page('Identify the Problem', 'Why We Need New Drive Software', [
    H('The problem'),
    P('The "library" is the layer of code between the robot\'s sensors and its motors: it is what turns "drive to the '
      'goal" into the right voltage on every wheel, 100 times a second. Last season we used two libraries written by '
      'other teams and neither fit us.'),
    TABLE([
        ['What happened last season', 'What it cost us'],
        ['Started on EZ-Template, which only drives by distance and angle', 'Every autonomous was a chain of "drive 24, turn 90". One slip and everything after it was off.'],
        ['Switched to LemLib halfway through the season, in a rush', 'We could not get its tracking wheel offsets calibrated, so the robot\'s idea of where it was drifted.'],
        ['Neither library supports our new drivetrain (page D-41)', 'A mecanum drive with two extra center wheels has six motors that must be mixed. Both libraries assume a tank drive.'],
        ['When a match went wrong we had no record of why', 'We guessed. Fixes were based on what we thought we saw.'],
    ], widths=[3.3, 4.6]),
    H('Criteria: what the software must do'),
    B([
        '**Drive our robot:** tank drives and holonomic drives, including the six-motor "asterisk" layout.',
        '**Know where it is:** track the robot\'s position on the field with whatever sensors are mounted.',
        '**Go where it is told:** drive to a point, to a point facing a direction, and along a path.',
        '**Tune itself:** find its own control settings, so a rebuilt robot is ready in minutes, not days.',
        '**Explain itself:** record what happened in a match so decisions come from data.',
    ]),
    H('Constraints: the limits we work inside'),
    TABLE([
        ['Constraint', 'Why it matters'],
        ['Runs on the V5 Brain with PROS 4.2.2', 'Fixed processor speed and memory. No extra computers are legal on the robot.'],
        ['One programmer', 'Everything must be testable without the robot, because the robot is usually being built.'],
        ['The team has one or two field walls, not a full field', 'Calibration and testing cannot assume a full 12 ft field.'],
        ['Must last two seasons', 'Nothing in the library may be specific to this year\'s game.'],
        ['All code is ours to explain', 'Judges and teammates must be able to ask "why" about any line.'],
    ], widths=[3.3, 4.6]),
], LEMLIB + '; ' + EZ + '; 96671H 2025-26 notebook (Stratagem V2.0 code).', R)

page('Identify the Problem', 'What Our Robot Asks of the Software', [
    P('Before researching solutions we listed what this season\'s robot physically needs, then turned each need into a '
      'software requirement. This is the list every later decision in this section is checked against.'),
    TABLE([
        ['What the robot has or must do', 'What the software therefore needs', 'How we will measure it'],
        ['Mecanum drive with two straight center wheels ("asterisk"), six motors', 'A wheel mixer for 4 corners + 2 centers, driving, strafing and turning at once', 'All six motors share the load in a straight drive and in a turn'],
        ['Drivers want "forward" to always mean away from them', 'Field-centric control using the inertial sensor', 'Robot drives the same direction whichever way it faces'],
        ['Autonomous must end where the next motion expects', 'Position tracking (odometry) that can be corrected mid-routine', 'Under 1 in from the true position at the end of a routine'],
        ['A lift, a claw and an intake, but only 12 buttons', 'A macro system: one button can mean different things in different states', 'Every control reachable without letting go of the sticks'],
        ['Motors overheat in long matches', 'Notice heat and compensate before the robot drifts', 'A strafe stays straight with one hot motor'],
        ['Robot is rebuilt several times a season', 'Automatic tuning and calibration', 'New chassis tuned in one practice, not a week'],
        ['Failures must be explained', 'A log on the SD card and a tool to read it', 'Every match leaves a file we can replay'],
    ], widths=[2.6, 2.9, 2.4], size=10),
    BOX('**Why this page exists:** when we score options in a decision matrix later, the criteria and their weights come '
        'from this table, not from what is easiest to program.'),
    H('What "holonomic" means'),
    P('A tank drive can only go forward, backward and turn. A holonomic drive can also slide sideways (strafe) without '
      'turning, and can do all three at once. Our drive base testing (Section D) chose a mecanum drive with two extra '
      'straight wheels in the middle for pushing power. Seen from above the six wheels look like an asterisk (*), so '
      'that is what we call it.'),
], '96671H notebook, Section D (drive base testing); VEX V5RC 2026-27 Game Manual; ' + PROS + '.', R, key='needs')

page('Research', 'Existing Libraries', [
    P('We started by studying the libraries other teams publish, to learn what a good one contains and to decide '
      'whether one of them could be adapted. For each we read the documentation and the source code.'),
    TABLE([
        ['Library', 'Drive types', 'Position tracking', 'Tuning tools', 'What we learned from it'],
        ['LemLib', 'Tank only', 'Tracking wheels + inertial sensor', 'None built in', 'The moveToPoint(x, y) style of command, the "boomerang" curve, clear code comments'],
        ['EZ-Template', 'Tank only', 'Added recently; built around distance and angle', 'On-screen PID tuner, auton selector', 'Every motion needs an exit condition and a timeout; a brain-screen tuner saves hours'],
        ['OkapiLib', 'Tank, X-drive, mecanum', 'Basic', 'None', 'Holonomic "chassis models"; but it is no longer maintained for the current PROS'],
        ['JAR-Template', 'Tank and holonomic', 'Tracking wheels', 'None', 'Holonomic odometry is practical in VEX; but it is for VEXcode, not PROS'],
    ], widths=[1.1, 1.05, 1.5, 1.35, 2.9], size=10),
    H('What none of them had'),
    B([
        'A holonomic drive with **extra center wheels**. Every library assumes four corner wheels at most.',
        '**One** library with position tracking, holonomic driving, self-tuning and match logging together. Teams '
        'combine two or three libraries, and the pieces do not share information.',
        'A way to **test without the robot**. Each library can only be tried by downloading to a real robot.',
    ]),
    H('Reflection'),
    P('Reading the source changed our plan. We had assumed a library was mostly "the math". In both LemLib and '
      'EZ-Template, the math is a small part: most of the code handles things going wrong (a sensor unplugged, a motion '
      'that never finishes, two tasks touching the same value). We added "what happens when something fails" as a '
      'question to ask of every feature we write.'),
], LEMLIB + '; ' + EZ + '; OkapiLib, okapilib.github.io/OkapiLib; JAR-Template (GitHub).', R)

page('Research', 'Where Each Technique Comes From', [
    P('Almost nothing in a drive library is new. The skill is choosing well-proven techniques and fitting them '
      'together. This table lists every outside idea SapphireLib builds on and where it is used, so credit is given '
      'where the idea appears.'),
    TABLE([
        ['Source', 'What we took from it', 'Used in'],
        ['EZ-Template', 'Exit conditions and timeouts on every motion; brain-screen tuner', 'Motions, Brain GUI'],
        ['LemLib', 'moveToPoint / moveToPose command style; boomerang; comment style', 'Motions'],
        ['5225A PiLons position tracking paper', 'The arc method for tracking wheels', 'Odometry'],
        ['Purdue SIGBots Wiki', 'Using the inertial sensor for heading; PID basics', 'Odometry, PID'],
        ['Game Manual 0 (FTC)', 'Mecanum wheel mixing and field-centric driving', 'Chassis'],
        ['Coulter 1992 (Carnegie Mellon)', 'Pure pursuit path following', 'Path following'],
        ['Astrom and Murray, Feedback Systems', 'PID theory, pole placement, phase margin', 'PID, Auto-Tune'],
        ['WPILib (FRC) system identification', 'The kS / kV / kA motor model', 'Auto-Tune'],
        ['Thrun, Burgard and Fox, Probabilistic Robotics', 'Monte Carlo localization', 'Localization'],
        ['PROS and LVGL documentation', 'Tasks, devices, SD card, the brain screen', 'Everything'],
    ], widths=[2.9, 3.4, 1.6]),
    H('How we used sources'),
    B([
        'We **read the idea, then wrote our own code** for it. No file in SapphireLib is copied from another library.',
        'Where two sources disagreed (for example, three tracking wheels against two plus an inertial sensor) we '
        'treated it as a design choice and made a decision matrix.',
        'Each page that uses a source names it in the red box at the bottom of that page.',
    ]),
], LEMLIB + '; ' + EZ + '; ' + PILONS + '; ' + SIGBOTS_ODOM + '; ' + GM0 + '; ' + COULTER + '; ' + ASTROM + '; WPILib docs, docs.wpilib.org; S. Thrun et al., Probabilistic Robotics (MIT Press, 2005); ' + PROS + '; LVGL 9.2 docs, docs.lvgl.io/9.2.', R)

page('Brainstorm and Pick Best Solution', 'Three Ways Forward', [
    P('From the research we saw three realistic approaches. We wrote down what each would look like in practice before '
      'scoring them.'),
    TABLE([
        ['', 'A. Keep last year\'s code on LemLib', 'B. Go back to EZ-Template', 'C. Write our own: SapphireLib'],
        ['What we would do', 'Keep Stratagem V2.0 and add a holonomic layer inside LemLib\'s code', 'Use EZ-Template\'s drive and add holonomic control and odometry to it', 'Start from an empty PROS project and write every part'],
        ['First drive', 'Same day', 'Same day', 'Two to three weeks'],
        ['Asterisk drive', 'We edit someone else\'s tank-drive code', 'We edit someone else\'s tank-drive code', 'Designed in from the first line'],
        ['When it breaks at a tournament', 'Debug thousands of lines we did not write', 'Debug thousands of lines we did not write', 'We wrote it, so we know where to look'],
        ['Tools (tuning, logging)', 'Add separately', 'Tuner exists; logging added separately', 'Built to share data with each other'],
        ['Biggest risk', 'Library updates overwrite our edits', 'Odometry is not its strength', 'One programmer; nothing works until a lot works'],
    ], widths=[1.5, 2.1, 2.1, 2.2], size=10, bold_first_col=True),
    H('How the approaches differ'),
    P('A and B are fast today and slow later: the robot drives immediately, but every feature the robot needs that the '
      'library lacks has to be forced into code designed for a different drivetrain. C is slow today and fast later: '
      'nothing drives for weeks, but each feature is built for our robot and for testing on a laptop.'),
    BOX('**What would change our mind:** if the season started in two weeks, A wins. Because we started in June with '
        'the whole summer ahead and the robot not yet built, time to first drive matters less than usual. That is why '
        '"works on day one" gets a middle weight, not the top weight, on the next page.'),
], LEMLIB + '; ' + EZ + '; 96671H 2025-26 notebook (Stratagem V2.0 code).', R)

_crit = [('Supports the six-motor asterisk drive', 5), ('Any mix of tracking sensors, easy to calibrate', 4),
         ('We can explain and fix every line', 4), ('Tuning and logging tools built in', 3),
         ('Works on day one', 3), ('Low risk with one programmer', 2)]
_scores = [[1, 1, 5], [3, 2, 5], [2, 2, 5], [2, 3, 5], [5, 5, 1], [4, 4, 2]]
page('Brainstorm and Pick Best Solution', 'Library Decision Matrix', [
    P('Each approach is scored from 1 (poor) to 5 (excellent) on each criterion. The score is multiplied by the '
      'criterion\'s weight (1 to 5, from how much our robot depends on it, page [[needs]]). The number in brackets is the '
      'weighted score. Green is the best in a row, red the worst.'),
    MATRIX(_crit, ['A. Keep LemLib', 'B. EZ-Template', 'C. SapphireLib'], _scores),
    H('Why each score'),
    B([
        '**Asterisk drive:** A and B are tank-only, so both score 1. This is the highest weight because without it the '
        'robot from Section D cannot drive at all.',
        '**Sensor mix:** LemLib requires tracking wheels (3). EZ-Template\'s tracking is newer and less flexible (2).',
        '**Explain every line:** we scored ourselves honestly at 2 for both outside libraries after last season, when '
        'we could not find why LemLib\'s offsets would not calibrate.',
        '**Day one** and **risk:** the outside libraries win clearly. Writing our own is a real risk and scores 1 and 2.',
    ]),
    BOX('**Decision: write our own library, SapphireLib.** It scores highest because the two criteria our '
        'robot depends on most cannot be met by the others. Decided by Cody Paschal (programmer). To manage the risk we will (1) build in phases so the robot can drive '
        'after phase 1, and (2) keep Stratagem V2.0 working as a fallback until SapphireLib has driven an autonomous.',
        fill='FFF2CC'),
], LEMLIB + '; ' + EZ + '; criteria from page G-1 and G-2.', R)

page('CAD/Design and Prototype', 'Designing the Library Before Writing It', [
    P('A library is designed like a mechanism: parts with clear jobs that connect in known ways. Before any code we '
      'drew the layers below. Each layer only talks to the one beneath it, so a part can be rebuilt without touching '
      'the rest.'),
    IMGB('architecture.png', caption='SapphireLib\'s layers. The top layer is our robot; everything below it works on any V5 robot.'),
    H('Design rules we set'),
    B([
        '**Tell it what sensors exist, it picks the math.** No tracking wheels, one, or two: the same library.',
        '**Math is separate from hardware.** Every calculation is a plain function that takes numbers and returns '
        'numbers. That lets us test it on a laptop with no robot (page [[unittests]]).',
        '**Nothing may freeze the robot.** Every wait has a timeout. Every motion reports whether it finished or gave up.',
        '**Tools are optional.** A team that does not want our screen or logger never has to turn them on.',
    ]),
    H('Phases'),
    TABLE([
        ['Phase', 'What it delivers', 'Robot can then...'],
        ['1. Chassis control', 'PID, tank and holonomic drives, driver control', 'Drive straight and turn to an angle'],
        ['2. Odometry', 'Position tracking with any sensor mix', 'Know where it is'],
        ['3. Motions', 'Move to point, move to pose, follow a path', 'Run a full autonomous'],
        ['4. Tools', 'Brain screen, tuners, SD card logging', 'Be tuned and debugged quickly'],
        ['5. Mechanisms', 'Lifts, pistons, controller macros', 'Run the whole robot on one library'],
    ], widths=[1.6, 3.5, 2.8]),
    P('Phase 1 comes first on purpose: the robot can drive in practice before position tracking exists.'),
], ASTROM + '; ' + LEMLIB + ' (project layout); ' + PROS + '.', R, key='design')

page('Monthly Gantt Charts', 'SapphireLib Plan', [
    IMGB('gantt_plan.png'),
    H('Overall goal'),
    P('Have SapphireLib driving the competition robot in autonomous by mid-September, before our first '
      'tournament.'),
    H('Sub-goals'),
    B([
        'June: finish research and the decision (done), then the PID controller and both drive types.',
        'Late June to mid-July: odometry for every sensor combination, then the pose motions.',
        'Late July: the brain screen and an automatic tuner, so the build team can tune without the programmer.',
        'August: driver macros and SD card logging.',
        'Late August to mid-September: move everything onto the competition robot.',
    ]),
    H('Top priorities'),
    P('Phases 1 to 3 (drive, track, move). If these slip, the fallback is last year\'s Stratagem code for the first '
      'tournament.'),
    H('Extra goals'),
    P('If we are ahead: a way to correct position drift during a match using distance sensors. Research only for now.'),
    H('Reasons for gaps'),
    P('The same person programs and helps with drive base testing (Section D), so programming days in June and July '
      'are the days the test drive bases are not being run.'),
    BOX('See page [[gantt_mid]] for progress against this plan part way through, and page [[gantt_final]] for the final chart.'),
], '96671H notebook, page A-4 (How to Read Gantt Charts); SapphireLib docs/ROADMAP.md.', R, key='gantt_plan')

# ------------------------------------------------------------------ core control

page('Research', 'How a PID Controller Works', [
    P('Every motion the robot makes on its own comes down to one question, asked 100 times a second: "how many volts '
      'should the motors get right now?" A PID controller answers it. Cruise control in a car is the everyday example: '
      'it compares the speed you set to the speed you are going and adjusts the engine.'),
    IMGB('pid_loop.png', caption='The loop: compare target to reality, combine three reactions, send volts, measure again.'),
    TABLE([
        ['Part', 'In plain words', 'What it fixes', 'Too much of it causes'],
        ['P (proportional)', 'Push harder the farther away you are', 'Gets the robot moving toward the target', 'Overshooting and bouncing back'],
        ['I (integral)', 'If you have been a little off for a while, push more', 'The last small error friction leaves behind', 'A big overshoot after being blocked ("windup")'],
        ['D (derivative)', 'If you are closing in fast, ease off', 'Overshoot: it is the brake', 'Jitter from sensor noise'],
    ], widths=[1.45, 2.3, 2.25, 1.9]),
    H('What we decided to add to the textbook version'),
    P('Reading EZ-Template, LemLib and the control textbook showed four known weaknesses of a basic PID. We chose to '
      'build a fix for each into ours from the start, then test that each one earns its place (next three pages).'),
    TABLE([
        ['Weakness', 'Our addition'],
        ['Integral keeps growing while the robot is stuck', 'Anti-windup: stop adding to I while the output is already at its limit'],
        ['Changing the target makes D spike', 'Option to take D from the measurement instead of the error'],
        ['0 to 12 V instantly slips the wheels', 'Slew limit: cap how fast the output may change'],
        ['Gains only work at one loop speed', 'Gains are per second, not per loop'],
    ], widths=[3.4, 4.5]),
], SIGBOTS_PID + '; ' + ASTROM + ', ch. 10 (PID Control); L. Llamas, "The PID Controller," luisllamas.es; ' + EZ + ' (PID tuning guide).', C)

page('Programming', 'Our PID Code', [
    P('This is the heart of the controller, trimmed to its key lines. Green text after // is a comment: a note to the '
      'reader that the robot ignores. Every motion, the lift and the heading hold all run through this one function.'),
    CODE(r'''
double PID::step(double target, double measurement, double dtS) {
    const double error = target - measurement;      // how far off are we?

    // I: add up error over TIME (dtS = seconds since last loop), so kI is "per second"
    const double integralDelta = error * dtS;
    integral_ += integralDelta;

    // D: how fast the error is changing, per second. Optionally from the measurement,
    // so that changing the target does not look like a sudden huge change
    double derivative = (config_.derivativeOnMeasurement ? -(measurement - prevMeasurement_)
                                                         : (error - prevError_)) / dtS;

    double output = config_.gains.kP * error + config_.gains.kI * integral_
                  + config_.gains.kD * derivative;

    // anti-windup: already past the limit and still pushing the same way?
    // Then undo this loop's integration. It could only come back later as overshoot
    if (std::fabs(output) > config_.outputLimit && (output > 0.0) == (integralDelta > 0.0)) {
        integral_ -= integralDelta;
        output = config_.gains.kP * error + config_.gains.kI * integral_
               + config_.gains.kD * derivative;
    }

    // slew: the output may only change by so much per loop (no 0 V to 12 V jumps)
    output = prevOutput_ + std::clamp(output - prevOutput_, -config_.slewRate, config_.slewRate);
    return std::clamp(output, -config_.outputLimit, config_.outputLimit);   // a motor has 12 V
}
'''),
    H('How this code was developed'),
    B([
        '**Version 1 (June)** was the textbook: three lines for P, I and D added per loop.',
        '**Version 2** added the output limit and the slew limit, because a jump from 0 V to 12 V in one loop slips the wheels.',
        '**Version 3** changed I and D to be per second. This came from a bug found later while building the automatic '
        'tuner (page [[autotune_code]]): gains worked out by a formula were wrong by exactly the loop time, with kI 100 times too '
        'large and kD 100 times too small at a 10 ms loop. Making the gains per second fixed it for good.',
        '**Version 4** added the "observer": after each step the controller can hand its numbers to the SD card logger.',
    ]),
    BOX('**Why one PID for everything:** a fix made here reaches every mechanism at once, and one set of tests (next '
        'pages) covers them all.'),
], SIGBOTS_PID + '; ' + ASTROM + ', sec. 10.4 (Integrator Windup); SapphireLib src/sapphirelib/control/pid.cpp.', C)

page('Test', 'PID Test 1: Anti-Windup', [
    H('Question'),
    P('Does our anti-windup rule actually stop the overshoot that happens after the robot is blocked?'),
    H('Procedure (repeatable)'),
    B([
        'Tool: the library\'s PID running against the simulated forward axis of our drive (the motor model measured by '
        'Auto-Tune: 1.0 V friction, 0.20 V per in/s, 0.045 V per in/s^2, 30 ms delay).',
        'Target: drive 24 in. For the first 1.5 s a wall at 3 in stops the robot, as if pinned by an opponent. Then the '
        'wall is removed.',
        'Gains: kP 0.9, kI 1.5, kD 0.06. Loop: 10 ms. Run for 9 s.',
        'Run twice: once with anti-windup off, once with it on. Nothing else changes.',
        'Measure: the farthest the robot goes past 24 in (overshoot).',
    ]),
    IMGB('pid_windup.png'),
    TABLE([
        ['', 'Anti-windup off', 'Anti-windup on'],
        ['Overshoot past the 24 in target', '%.1f in' % WIND['none']['overshootIn'], '%.1f in' % WIND['guard']['overshootIn']],
        ['Time after release to stay within 0.5 in', '%.1f s' % WIND['none']['settleS'], '%.1f s' % WIND['guard']['settleS']],
    ], widths=[3.9, 2.0, 2.0], fills={(1, 1): RED, (1, 2): GREEN, (2, 1): RED, (2, 2): GREEN}),
    H('What we saw'),
    P('While held against the wall the "off" controller kept adding to its integral for 1.5 s. When released it drove '
      'far past the target before that stored-up push drained away. In a match that is a robot driving through the '
      'goal it was meant to stop at. With anti-windup on, the overshoot shrank by about %d%%.'
      % round(100 * (1 - WIND['guard']['overshootIn'] / WIND['none']['overshootIn']))),
    BOX('**Decision from this test:** anti-windup is turned on automatically whenever an output limit is set, which is '
        'every controller on our robot. Decided by Cody Paschal from the data above.', fill='FFF2CC'),
    P('**Reflection:** the remaining %.1f in overshoot comes from the integral gathered in the first instant after '
      'release. It pushed us toward controllers that need no I term at all, which the Auto-Tune work later delivered.'
      % WIND['guard']['overshootIn']),
], ASTROM + ', sec. 10.4; SapphireLib simulator (tools/analyzer/js/model.js), run by notebook/build/experiments.js.', C, key='windup')

page('Test', 'PID Tests 2 and 3: Derivative Kick and Loop Speed', [
    SIDE('pid_kick.png', 3.7, [
        H('Test 2: changing the target'),
        P('**Procedure:** heading-hold controller on the simulated turn axis. At 0.2 s the target heading is nudged by '
          '5 degrees, as when the driver taps the turn stick. Run once with D taken from the error, once from the '
          'measurement. Measure the largest voltage asked for.', 11.5),
        P('**Result:** D on error asked for **%.0f V**, almost five times what a motor has. D on measurement asked for '
          '**%.1f V**. The robot ends at the same heading either way.' % (KICK['onError']['peakVolts'], KICK['onMeasurement']['peakVolts']), 11.5),
    ], img_left=True),
    SIDE('pid_period.png', 3.7, [
        H('Test 3: changing the loop speed'),
        P('**Procedure:** one set of gains (kP %.2f, kI %.1f, kD %.2f), 24 in drive. Run at a 10 ms loop and a 20 ms '
          'loop with our per-second gains. Then run how an ordinary per-loop PID tuned at 10 ms behaves at 20 ms and '
          'at 5 ms. Measure settle time and overshoot.' % (PERIOD['gains']['kP'], PERIOD['gains']['kI'], PERIOD['gains']['kD']), 11.5),
    ], img_left=True),
    TABLE([['Controller and loop speed', 'Settles within 0.5 in after', 'Overshoot']] +
          [[c['name'], '%.2f s' % c['settleS'], '%.2f in' % c['overshootIn']] for c in PERIOD['cases']],
          widths=[3.9, 2.2, 1.8], fills={(1, 2): GREEN, (2, 2): GREEN, (3, 2): YELLOW, (4, 2): RED}),
    H('What the data says'),
    B([
        'With per-second gains, doubling the loop time changed the settle time by %.2f s. The gains survive.'
        % abs(PERIOD['cases'][1]['settleS'] - PERIOD['cases'][0]['settleS']),
        'With per-loop gains the same robot overshoots %.2f in at 5 ms and takes %.2f s to settle at 20 ms. A tuning '
        'done at one loop speed is wrong at another.' % (PERIOD['cases'][3]['overshootIn'], PERIOD['cases'][2]['settleS']),
    ]),
    BOX('**Decisions (Cody Paschal):** (1) Controllers whose target moves while running (driver heading hold) use D on '
        'measurement. (2) All gains are per second. This matters to us because the driver loop runs at a different '
        'speed than the autonomous loop, and both share controllers.', fill='FFF2CC'),
], ASTROM + ', sec. 10.5 (Implementation: derivative action, setpoint weighting); ' + SIGBOTS_PID + '; SapphireLib simulator, run by notebook/build/experiments.js.', C)

page('Programming', 'Driving Six Wheels: the Holonomic Mixer', [
    P('The driver gives three wishes at once: go forward, slide sideways, and turn. The "mixer" turns those three '
      'wishes into a voltage for each wheel (shown simplified below). Mecanum wheels have angled rollers, so spinning the corners in different '
      'directions makes the robot slide sideways.'),
    IMGB('wheel_mix.png'),
    CODE(r'''
// mix the three wishes into the four corner wheels
frontLeft  = throttle + strafe + turn;
frontRight = throttle - strafe - turn;
backLeft   = throttle - strafe + turn;
backRight  = throttle + strafe - turn;

// if any wheel was asked for more than 12 V, shrink all four by the same amount,
// so the robot still moves in the direction asked, just slower
const double biggest = std::max({fabs(frontLeft), fabs(frontRight), fabs(backLeft), fabs(backRight)});
if (biggest > 12.0) { scale all four by 12.0 / biggest; }

// the two straight center wheels cannot push sideways. Work backward from the four
// corners to find how much "forward" and "turn" was asked, and give them only that
const double throttleVolts = (frontLeft + frontRight + backLeft + backRight) / 4.0;
const double turnVolts     = (frontLeft - frontRight + backLeft - backRight) / 4.0;
middleLeft  = throttleVolts + turnVolts;
middleRight = throttleVolts - turnVolts;
'''),
    H('Design choices'),
    B([
        '**One mixer for mecanum and X-drive.** Both use the same plus and minus pattern, so the library supports '
        'either with the same code, including the X-drive and mecanum test bases from Section D.',
        '**Center wheels are worked out from the corners, not from the sticks.** Whatever asked for the motion (driver, '
        'autonomous, the tuner), the center wheels react the same way. This removed a whole class of "works in driver, '
        'not in auton" bugs.',
        '**Shrink together, never clip one wheel.** Clipping one wheel at 12 V changes the direction of travel.',
    ]),
], GM0 + '; OkapiLib X-drive model, okapilib.github.io/OkapiLib; SapphireLib src/sapphirelib/chassis/holonomic_drivetrain.cpp.', C)

page('Programming', 'Field-Centric Driving and the Joystick Curve', [
    SIDE('joystick_curve.png', 3.3, [
        H('Joystick curve'),
        P('A straight-line stick makes small, careful movements hard: a tiny push already moves the robot quickly. The '
          'curve blends a straight line with a cubic (x times x times x), so the first half of the stick gives fine '
          'control and the top end still reaches full speed.', 11.5),
        CODE('return curve * x * x * x + (1.0 - curve) * x;', 8.5),
        P('The driver picks the blend. A deadband ignores the one or two counts a stick reads at rest.', 11.5),
    ]),
    H('Field-centric ("headless") driving'),
    P('With a robot that can slide in any direction, "forward on the stick" is confusing once the robot has turned '
      'around. In field-centric mode, pushing the stick away from the driver always moves the robot away from the '
      'driver. The inertial sensor tells us how far the robot has turned, and we rotate the stick by that angle:'),
    CODE(r'''
// rotate the driver's stick by how far the robot has turned since "forward" was set
const double headingRad = (robotHeading - fieldForwardHeading) * (kPi / 180.0);
const double robotThrottle =  throttle * cos(headingRad) + strafe * sin(headingRad);
const double robotStrafe   = -throttle * sin(headingRad) + strafe * cos(headingRad);
holonomic(robotThrottle, robotStrafe, turn);   // then mix as on the previous page
'''),
    H('Problems this solves for the drivers'),
    TABLE([
        ['Situation', 'Problem', 'What the library provides'],
        ['Robot has spun around', '"Forward" on the stick now drives toward the driver', 'Field-centric mode'],
        ['Lining up on a goal', 'A straight-line stick is too sensitive near the center', 'The curve; the driver chooses how much'],
        ['Autonomous ended at an angle', '"Forward" would point wherever autonomous left the robot', 'resetFieldHeading() re-zeroes "forward"'],
        ['Stick not perfectly centered', 'Robot creeps when nobody is touching it', 'Deadband'],
    ], widths=[2.3, 3.2, 2.4]),
    P('**Reflection:** the rotation is four lines of math. The rest of this page exists because a driver feature '
      'is only finished when it handles the awkward moments of a match: after a spin, after autonomous, and with a '
      'worn controller.'),
], GM0 + ' (field-centric control); ' + EZ + ' (joystick curves); SapphireLib src/sapphirelib/control/joystick_curve.cpp.', C)

page('Test', 'Testing Code Without a Robot', [
    H('The problem'),
    P('The robot was being designed and rebuilt all summer, and a bug that only shows up on the robot can take a whole '
      'practice to find. We needed to test the library on a laptop.'),
    H('Our method: desktop unit tests'),
    P('Because every calculation is a plain function (design rule 2, page [[design]]), we can call it on a laptop with numbers '
      'we choose and check the answer against one worked out by hand. Each check is a "unit test". They run in seconds, '
      'and they run again automatically every time the code changes.'),
    TABLE([
        ['Area', 'Example of what a test checks', 'Why it matters'],
        ['PID', 'Integral does not grow while the output is at its limit', 'Page [[windup]]'],
        ['Angles', '350 degrees to 10 degrees is a 20 degree turn, not 340', 'Robot turns the short way'],
        ['Odometry', 'Spinning in place moves the pose by zero', 'Wheel offsets are applied correctly'],
        ['Path following', 'The lookahead point is on the path, ahead of the robot', 'Robot does not drive backward along a path'],
        ['Tuner', 'Recovers known motor values from simulated data with noise', 'Auto-Tune can be trusted'],
        ['Localization', 'Same random seed gives exactly the same result', 'Tests are repeatable'],
        ['Telemetry', 'Log text matches the file format byte for byte', 'Old logs stay readable'],
    ], widths=[1.4, 4.0, 2.5]),
    TABLE([
        ['Test group', 'Count'],
        ['Desktop C++ test files, one per module', '48'],
        ['Simulator test files (JavaScript)', '10'],
        ['Run automatically on every code change', 'Yes (GitHub Actions)'],
    ], widths=[5.4, 2.5]),
    H('What unit tests cannot do'),
    P('They prove the math does what we intended. They cannot prove we intended the right thing, or that the real '
      'robot behaves like our model of it. For that we use the simulator (page [[simulator]]) and then the robot itself.'),
    BOX('**Reflection:** writing a test takes about as long as writing the function, which felt slow at first. It is '
        'also the only way one programmer can keep changing a library this size without breaking what already worked. '
        'From here on, each page that says "desktop test" means this.'),
], 'GitHub Actions docs, docs.github.com/actions; ' + LEMLIB + ' (CI layout); SapphireLib tests/ folder and .github/workflows/build.yml.', C, key='unittests')

# ------------------------------------------------------------------ odometry and motions

page('Research', 'How Odometry Works', [
    P('Odometry is how the robot knows where it is. It is dead reckoning, the way a ship navigated before GPS: start '
      'from a known spot, and keep adding up every small movement.'),
    SIDE('odom_diagram.png', 3.5, [
        H('The sensors'),
        B([
            '**Tracking wheels:** small unpowered wheels on rotation sensors that roll along the floor. One faces '
            'forward (vertical), one sideways (horizontal).',
            '**Inertial sensor (IMU):** reports which way the robot is facing.',
        ], 11.5),
        H('Every 10 ms'),
        B([
            'Read how far each wheel rolled.',
            'Subtract the part that came from the robot turning (the arc).',
            'Rotate what is left by the heading, and add it to the position.',
        ], 11.5),
    ]),
    H('The arc problem'),
    P('A tracking wheel is never exactly at the center of the robot. When the robot spins in place it has not travelled '
      'anywhere, but an off-center wheel rolls along a circle. The 5225A PiLons paper gives the fix: that false rolling '
      'equals the wheel\'s distance from the center (its offset) times the angle turned. Subtract it and only real '
      'travel remains.'),
    H('Two ways to get the heading'),
    TABLE([
        ['Method', 'How', 'Trade-off'],
        ['PiLons (2018)', 'Two parallel wheels; heading from the difference between them', 'Three wheels total. Heading error grows with wheel error.'],
        ['SIGBots Wiki', 'The inertial sensor reports heading directly', 'One fewer wheel. IMU drifts slowly over a match.'],
    ], widths=[1.4, 3.6, 2.9]),
    P('**What we learned:** odometry only ever adds. Any error (a wheel that slips, a bump from another robot) stays in '
      'the position for the rest of the match. That weakness is why this section later adds a second system to check '
      'odometry against the field walls.'),
], PILONS + '; ' + SIGBOTS_ODOM + '; ' + LEMLIB + ' (tracking wheel setup).', K)

_ocrit = [('Tracks sideways sliding (needed for holonomic)', 5), ('Not fooled by drive wheel slip', 4),
          ('Heading accuracy', 4), ('Space it takes inside the robot', 3), ('Ports and parts used', 2)]
_oscores = [[5, 5, 1, 1], [5, 5, 4, 1], [3, 5, 5, 5], [1, 3, 4, 5], [2, 3, 4, 5]]
page('Brainstorm and Pick Best Solution', 'Which Sensors for Odometry', [
    P('The research gave four ways to build odometry. We compared them for our robot, then made a second decision about '
      'what the library should support.'),
    TABLE([
        ['Option', 'Sensors', 'How it tracks'],
        ['1. Three wheels', '2 vertical + 1 horizontal tracking wheel', 'Heading from the two vertical wheels (PiLons)'],
        ['2. Two wheels + IMU', '1 vertical + 1 horizontal + inertial sensor', 'Heading from the IMU, both directions from wheels'],
        ['3. One wheel + IMU', '1 vertical + inertial sensor', 'Forward only; assumes the robot never slides sideways'],
        ['4. Drive motors + IMU', 'Encoders inside the drive motors + inertial sensor', 'No extra parts; wheel slip counts as travel'],
    ], widths=[1.8, 3.0, 3.1]),
    MATRIX(_ocrit, ['3 wheels', '2 wheels + IMU', '1 wheel + IMU', 'Motors + IMU'], _oscores),
    H('Reasoning'),
    B([
        'Our drive slides sideways on purpose, so options 3 and 4 score 1 on the top criterion: they cannot see a strafe.',
        'Mecanum rollers slip constantly, so motor encoders (option 4) are a poor measure of real travel.',
        'Three wheels would need space inside the chassis that the lift and intake are using.',
    ]),
    BOX('**Decision 1 (robot):** two tracking wheels plus the inertial sensor. Decided by Cody Paschal; the pods '
        'themselves are documented on the Odometry Pods pages.', fill='FFF2CC'),
    BOX('**Decision 2 (library):** support all of options 2, 3 and 4 with the same code. The test drive bases in '
        'Section D had no tracking wheels at all, and a future tank robot will not need a horizontal wheel. The library '
        'picks the math from whichever sensors it is given.', fill='FFF2CC'),
], PILONS + '; ' + SIGBOTS_ODOM + '; VEX Rotation Sensor, vexrobotics.com/276-6050.html; VEX Inertial Sensor, vexrobotics.com/276-4855.html.', K)

page('Programming', 'Odometry Code', [
    P('The whole of odometry\'s math is this one function. It runs every 10 ms on its own background task, so the '
      'position keeps updating no matter what the rest of the program is doing.'),
    CODE(r'''
PoseDelta computeOdometryDelta(double lastHeadingDeg, double headingDeg,
                               double verticalDeltaIn, double horizontalDeltaIn,
                               double verticalOffsetIn, double horizontalOffsetIn) {
    const double dThetaDeg = wrapDegrees180(headingDeg - lastHeadingDeg);   // how far we turned
    const double dThetaRad = dThetaDeg * kDegToRad;

    // remove each wheel's arc from turning, leaving its real movement
    const double localForwardIn = verticalDeltaIn   - verticalOffsetIn   * dThetaRad;
    const double localLateralIn = horizontalDeltaIn - horizontalOffsetIn * dThetaRad;

    // rotate the robot's own movement onto the field, using the average heading
    // over the update (the robot was turning while it moved)
    const double avgHeadingRad = (lastHeadingDeg + dThetaDeg / 2.0) * kDegToRad;
    return PoseDelta{
        .dxIn = localForwardIn * sin(avgHeadingRad) + localLateralIn * cos(avgHeadingRad),
        .dyIn = localForwardIn * cos(avgHeadingRad) - localLateralIn * sin(avgHeadingRad),
    };
}
'''),
    H('One function, four sensor setups'),
    TABLE([
        ['Sensors on the robot', 'What the function is given'],
        ['IMU + drive motor encoders', 'Motor encoders as the vertical wheel; horizontal = 0'],
        ['IMU + vertical tracking wheel', 'The vertical wheel; horizontal = 0'],
        ['IMU + horizontal tracking wheel', 'Motor encoders as vertical; the horizontal wheel'],
        ['IMU + both wheels (our robot)', 'Both wheels'],
    ], widths=[3.2, 4.7], fills={(4, 0): GREEN, (4, 1): GREEN}),
    H('Things that went wrong and what we changed'),
    B([
        '**An unplugged rotation sensor moved the pose about 515,000 in.** A disconnected sensor reports a huge error '
        'value. The wheel now holds its last good reading until the sensor answers again.',
        '**One bad inertial sensor reading made the heading "not a number" for the rest of the program.** Bad readings '
        'are now skipped.',
        '**Setting the start pose ignored the heading.** A routine that started facing 270 degrees had its whole field '
        'rotated. Setting the pose now also sets the heading everywhere it is read.',
    ]),
], PILONS + ' (arc method); ' + SIGBOTS_ODOM + '; SapphireLib src/sapphirelib/odom/odometry_math.cpp.', K, key='odomcode')

page('Test', 'Calibrating Tracking Wheel Offsets', [
    H('Why this test'),
    P('Last season we could not get LemLib\'s offsets right by measuring with a ruler. Before trusting our own '
      'odometry we worked out how much a wrong offset costs, then designed a way to measure it without a ruler.'),
    SIDE('offset_error.png', 3.9, [
        P('**Calculation:** false travel = offset error x angle turned (in radians). The chart shows it for three '
          'amounts of turning.', 11.5),
        P('A ruler measurement to the middle of a wheel inside a chassis is easily 0.25 in off. A routine with three '
          'full turns then carries **4.7 in** of error that never happened.', 11.5),
    ]),
    H('Our calibration procedure (repeatable)'),
    B([
        'Place the robot on the tiles. On the brain screen, Odom tab, press Calibrate Offsets.',
        'The robot spins in place. It has not travelled, so everything a wheel rolled is arc.',
        'Offset = distance the wheel rolled / angle turned. The screen shows both offsets.',
        'Copy them into the robot\'s config file. Repeat after any change to the pods.',
    ]),
    CODE('double calibrateTrackingWheelOffsetIn(double wheelDistanceIn, double rotatedRadians) {\n'
         '    return wheelDistanceIn / rotatedRadians;\n}'),
    H('Simulator test of the result'),
    TABLE([
        ['Setup', 'Odometry while the robot spins 3 s in place', 'Pass?'],
        ['Wheels mounted as the offsets say', 'Position stays still', 'Yes'],
        ['Vertical wheel on the opposite side', 'Position moves with every turn', 'No'],
    ], widths=[3.2, 3.5, 1.2], fills={(1, 2): GREEN, (2, 2): RED}),
    BOX('**Traceable change:** this test caught a real mistake. Our documentation said a positive vertical offset meant '
        '"right of center". The math treats positive as left. A hand-measured right-side wheel would have added about '
        '11 in of false travel per quarter turn. The spin calibration always gave the right sign, so we fixed the '
        'documentation and added a permanent test that pins both signs.', fill='FFF2CC'),
], PILONS + '; ' + LEMLIB + ' (tracking wheel offsets); SapphireLib tests/odom/odometry_math_test.cpp and tools/sim/test/sim.test.js.', K)

page('Programming', 'Motions and Knowing When to Stop', [
    P('A motion is a command like "drive to this point". The hard part is not starting it but deciding when it is '
      'done. Stop too early and the robot is short. Never stop and the whole autonomous freezes on one command.'),
    IMGB('flow_motion.png', caption='Every motion runs this loop every 10 ms. If the timeout runs out first, it returns "timed out".'),
    TABLE([
        ['Motion', 'What it does', 'Finished when'],
        ['driveDistance(24)', 'Straight line by distance, holding its heading', 'Within 1 in for 200 ms'],
        ['turnToHeading(90)', 'Spin to face a field direction', 'Within 2 degrees for 200 ms'],
        ['moveToPoint(x, y)', 'Go to a spot on the field', 'Within 1 in of the point'],
        ['moveToPose(x, y, heading)', 'Go to a spot and end facing a direction', 'Within 1 in and 2 degrees'],
        ['followPath(points)', 'Follow a curved route (next page)', 'Within 1 in of the last point'],
    ], widths=[2.3, 3.3, 2.3]),
    CODE(r'''
// a routine can ask each motion how it ended, and react
if (!drivetrain.moveToPoint(24, 48).settled()) {
    return;   // we hit something or got stuck: stop here instead of scoring in the wrong place
}
'''),
    H('How testing shaped this'),
    B([
        '**Motions used to return nothing.** A routine could not tell "arrived" from "ran out of time against a wall", '
        'and carried on blindly. Every motion now returns a result: settled, timed out or interrupted.',
        '**Leaving the exit settings empty meant a threshold of 0 in.** No robot can be 0 in from a target, so every '
        'motion ran to its timeout. The defaults are now 1 in and 2 degrees.',
        '**A motion running when autonomous ended kept driving into driver control.** Every motion now watches for the '
        'match mode changing and stops itself.',
    ]),
    P('**Reflection:** all three were found by running routines in the simulator and reading the log, not by '
      'inspection. Each is the kind of failure that costs a match and looks like "the robot just did something weird".'),
], EZ + ' (exit conditions); ' + LEMLIB + ' (moveToPoint, moveToPose); SapphireLib src/sapphirelib/motion/exit_tracker.cpp.', K)

page('Research', 'Pure Pursuit and Boomerang', [
    SIDE('pursuit_diagram.png', 3.6, [
        H('Pure pursuit: following a path'),
        P('Coulter (Carnegie Mellon, 1992) describes it like driving a car: you do not stare at the road under your '
          'bumper, you look some distance ahead and steer toward that.', 11.5),
        B([
            'Draw a circle around the robot. Its radius is the **lookahead**.',
            'Find where the circle crosses the path, ahead of the robot.',
            'Drive toward that point. Repeat every 10 ms.',
        ], 11.5),
    ]),
    P('Finding the crossing is a quadratic equation, the same one from algebra class: a line (the path segment) meets a '
      'circle (the lookahead). Our code solves it for each segment and keeps the crossing farthest along the path.'),
    SIDE('boomerang_diagram.png', 3.6, [
        H('Boomerang: arriving facing the right way'),
        P('A tank drive cannot slide sideways, so to arrive facing a direction it must curve in. LemLib\'s boomerang '
          'aims at a "carrot" placed behind the target. As the robot gets closer the carrot slides onto the target, '
          'bending the path into a smooth hook.', 11.5),
        P('carrot = target - (direction of final heading) x lead x distance left. Our lead is 0.6.', 11.5),
    ], img_left=True),
    H('What changes for a holonomic drive'),
    P('Our robot can slide sideways while it turns, so it does not need the carrot. Its moveToPose drives straight at '
      'the point and turns to the final heading at the same time. The library keeps boomerang for tank drives, so both '
      'drive types have a moveToPose.'),
    TABLE([
        ['Setting', 'Small value', 'Large value'],
        ['Lookahead distance', 'Hugs the path, but steers constantly and can wobble', 'Smooth, but cuts corners'],
        ['Boomerang lead', 'Nearly straight in; may arrive at the wrong angle', 'Wide hook; needs more room'],
    ], widths=[1.9, 3.1, 2.9]),
    P('Both settings are trade-offs with no "correct" answer in the papers, so we tested them (next two pages).'),
], COULTER + ', ri.cmu.edu; Purdue SIGBots Wiki, Basic Pure Pursuit, wiki.purduesigbots.com; ' + LEMLIB + ' (boomerang controller, lateral motion docs).', K)

_lrow = {r['lookaheadIn']: r for r in LOOK['rows']}
page('Test', 'Choosing the Lookahead Distance', [
    H('Question'),
    P('What lookahead distance follows a path most accurately without constant steering?'),
    H('Procedure (repeatable)'),
    B([
        'Simulator, our robot\'s measured drive model, perfect sensors (so only the path follower is being tested).',
        'One S-shaped path of 7 points across the field, about 230 in long, at up to 9 V.',
        'Run followPath() with lookahead 3, 6, 9, 12, 18 and 24 in. Nothing else changes.',
        'Measure every 10 ms: distance from the robot to the nearest point of the path. Also total the change in the '
        'robot\'s direction of travel ("steering"), and the time to finish.',
    ]),
    IMGB('lookahead.png'),
    TABLE([['Lookahead', 'Mean off path', 'Worst off path', 'Total steering', 'Time']] +
          [['%d in' % r['lookaheadIn'], '%.2f in' % r['meanIn'], '%.2f in' % r['worstIn'], '%d deg' % r['turningDeg'], '%.1f s' % r['timeS']] for r in LOOK['rows']],
          widths=[1.4, 1.6, 1.7, 1.7, 1.5],
          fills={(1, 3): RED, (5, 1): YELLOW, (5, 2): RED, (6, 1): RED, (6, 2): RED, (3, 0): GREEN, (4, 0): GREEN}),
    H('Analysis'),
    P('At 3 in the robot stays close to the line but steers %d degrees in total, more than twice the path\'s own '
      'turning: it is wobbling. At 24 in it is smooth and fast but cuts corners by up to %.1f in, enough to clip a '
      'field element. Between 9 and 12 in the error is about 1 in and the steering has levelled off.'
      % (_lrow[3]['turningDeg'], _lrow[24]['worstIn'])),
    BOX('**Decision (Cody Paschal):** default lookahead of 10 to 12 in for our robot. Each path may set its own, so a '
        'route through a tight gap can use 6 in and a long open run 18 in.', fill='FFF2CC'),
], COULTER + '; Purdue SIGBots Wiki, Basic Pure Pursuit; SapphireLib simulator (tools/sim), run by notebook/build/experiments.js.', K)

page('Test', 'Fixing Boomerang on a Tank Drive', [
    H('What we noticed'),
    P('In simulated tests the tank moveToPose sometimes arrived at the right spot facing the wrong way, or backed into '
      'the target. Trying a handful of poses by hand was not enough to see a pattern, so we tested hundreds.'),
    SIDE('tank_boomerang.png', 3.8, [
        H('Procedure'),
        B([
            'Desktop test. A simulated tank robot at the field center.',
            '300 random target poses (position and final heading).',
            'A pose "settles" if the robot ends within 1 in and 2 degrees before the timeout.',
            'Repeated with noise added to the odometry.',
        ], 11.5),
    ]),
    TABLE([
        ['', 'Settled, perfect odometry', 'Settled, noisy odometry'],
        ['Before the fix', '69 of 300 (23%)', '12 to 29 of 300'],
        ['After the fix', '299 of 300 (99.7%)', '297 or more of 300'],
    ], widths=[2.3, 2.8, 2.8], fills={(1, 1): RED, (1, 2): RED, (2, 1): GREEN, (2, 2): GREEN}),
    H('What the failures had in common'),
    B([
        'The controller chose "forward or reverse" again every loop. When the carrot swung past 90 degrees it flipped, '
        'and the robot backed in facing the wrong way.',
        'It steered at the carrot right to the end. Within an inch, the direction to the carrot is mostly sensor noise.',
        'Nothing ever turned it to the final heading that its own exit condition required.',
    ]),
    H('The changes, each tied to a failure above'),
    B([
        'Direction is chosen **once** at the start.',
        'Within 3 in of the target it heads for the point itself, not the carrot.',
        'It then turns in place to the final heading.',
    ]),
    BOX('**Reflection:** 23% is a number we would never have guessed from watching a few runs that looked fine. Testing '
        'in bulk with random inputs is now our standard for any motion. Limitation: this fix has been tested on a '
        'simulated tank only, because our robot this season is holonomic.'),
], LEMLIB + ' (boomerang controller); SapphireLib tests/motion/boomerang_math_test.cpp.', K)

page('Test', 'Motion Bug Log', [
    P('We keep one table of every motion bug: how it was found, what was wrong and what changed. It shows that each '
      'change to the motion code came from a test result, not a guess.'),
    TABLE([
        ['Found by', 'What the test showed', 'Cause', 'Change'],
        ['Simulator: square laps', 'followPath() on a lap ended instantly', 'A lap ends where it starts, so "near the last point" was true on the first loop', 'Also wait until pursuit reaches the last segment'],
        ['Desktop test: 500 random paths', 'Lookahead jumped to the lap\'s last side', 'The last side runs beside the first, inside the circle', 'Stop searching at the first miss after a hit'],
        ['Simulator: holonomic moves', 'Robot rotated during moveToPoint()', 'Turn command was zero, so nothing held the heading', 'Hold the starting heading with the turn PID'],
        ['Simulator: blocked path', 'Robot chased a blocked path for the rest of autonomous', 'Pursuit only ended by reaching the end', 'Timeout on the pursuit phase (10 s default)'],
        ['Simulator: far from the path', 'Robot drove straight to the last point, skipping the route', 'No part of the path was inside the lookahead circle', 'Go back to the nearest point of the path'],
        ['Code review + desktop test', 'driveDistance() jumped the pose backward', 'It zeroed the motor encoders odometry was also reading', 'Measure from wherever the encoders are'],
        ['Desktop test: empty path', 'Crash', 'Asked for the last point of an empty list', 'Return "aborted"'],
    ], widths=[1.55, 1.95, 2.3, 2.1], size=9.5),
    H('Checking the fixes did not change anything else'),
    P('For each fix we kept a copy of the old code inside the test and ran old and new side by side on hundreds of '
      'cases. On every case that used to work, the new code sends the same motor commands at the same times. Only the '
      'broken cases changed.'),
    BOX('**Reflection on the first three phases (planned for 07/15):** chassis control, odometry and the motions were '
        'written and passing their tests later than planned (see page [[gantt_mid]]), mostly because of the time spent on the '
        'bugs above. We judged that worth it: each of these would have been far harder to find on a robot at a '
        'tournament. The cost is that tools (phase 4) start later than the Gantt chart on page [[gantt_plan]] planned.'),
], COULTER + '; ' + LEMLIB + '; SapphireLib tests/motion/pure_pursuit_math_test.cpp, exit_tracker_test.cpp and tools/sim/test/sim.test.js.', K, key='buglog')
