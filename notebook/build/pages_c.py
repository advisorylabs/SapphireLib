"""Section G pages, part 3: Monte Carlo localization, then the competition robot and its autons."""
import json
import os

from nb import B, BOX, CODE, H, IMGB, MATRIX, P, SIDE, TABLE, Page, GREEN, RED, YELLOW, GRAY, BLUE
from pages_a import ASTROM, COULTER, EZ, LEMLIB, PROS, T

HERE = os.path.dirname(os.path.abspath(__file__))


def data(name):
    with open(os.path.join(HERE, 'data', name + '.json'), encoding='utf-8') as f:
        return json.load(f)


MVO = data('mcl_vs_odom')
DRIFT = data('drift_series')
PC = data('particle_count')
DIAG = data('diagonal')
CLAW = data('claw')

V = 'Competition robot + autons'
THRUN = 'S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics (MIT Press, 2005), ch. 8'
DELLAERT = 'F. Dellaert, D. Fox, W. Burgard and S. Thrun, "Monte Carlo Localization for Mobile Robots," IEEE ICRA 1999'
DIST = 'VEX V5 Distance Sensor, vexrobotics.com/276-4852.html'
GPS = 'VEX V5 GPS Sensor, vexrobotics.com/276-7405.html'

PAGES = []


def page(*args, **kwargs):
    PAGES.append(Page(*args, **kwargs))


odom_peak = max(s['e'] for s in DRIFT['odom'])
odom_end = DRIFT['odom'][-1]['e']
mcl_end = DRIFT['mcl'][-1]['e']
normal = [r for r in MVO['rows'] if r['condition'] == 'Normal']

# ------------------------------------------------------------------ localization

page('Identify the Problem', 'Odometry Drifts', [
    H('The problem'),
    P('Odometry only ever adds movements together (page [[odomcode]]). Nothing in it can notice that it has become '
      'wrong. Three things make it wrong in every match:'),
    TABLE([
        ['Cause', 'Size', 'Effect'],
        ['Tracking wheels read slightly long or short', 'A percent or two', 'Error grows with every inch driven'],
        ['The inertial sensor drifts', 'A fraction of a degree per minute', 'Every later movement is added in a slightly wrong direction'],
        ['Another robot bumps ours, or a wheel lifts', 'Inches, instantly', 'The wheels never see it, so the error is permanent'],
    ], widths=[3.2, 2.0, 2.7]),
    H('How big is it? (simulator test)'),
    P('**Procedure:** simulator, the Field tour routine (25 s, drives a lap of the field). Tracking wheels set 3% off '
      'with 2% slip and the inertial sensor drifting 3 degrees per minute ("worn" settings). At 6 s the robot is shoved '
      '6 in sideways, as in a collision. Measured every 100 ms: distance between where odometry says the robot is and '
      'where it really is.'),
    IMGB('drift_only.png'),
    TABLE([
        ['Measured', 'Odometry alone'],
        ['Largest error before the bump (first 6 s)', '%.1f in' % max(s['e'] for s in DRIFT['odom'] if s['t'] < 6)],
        ['Largest error in the routine', '%.1f in' % odom_peak],
        ['Error at the end of the routine', '%.1f in' % odom_end],
        ['Our criterion (page [[needs]])', 'Under 1 in'],
    ], widths=[4.4, 3.5], fills={(2, 1): RED, (3, 1): RED, (4, 1): GREEN}),
    P('Even with good wheels and no bump, the same routine ends %.1f in off on average (5 runs). A desktop test of a '
      'longer case (80 s of laps with a 4%% wheel error) ended 28.8 in off.' % normal[0]['odomEnd']),
    H('Why it matters'),
    P('The first motion of an autonomous is accurate. The tenth can be inches off, and a claw 2 in from where the '
      'program thinks it is misses the Pin. Our criterion needs something that can look at the field and correct the '
      'position **during** the routine.'),
    BOX('**Constraint carried from page G-1:** only legal V5 sensors, the V5 Brain\'s processor, and a practice area '
        'with one or two field walls.'),
], 'Purdue SIGBots Wiki, Odometry, wiki.purduesigbots.com/software/odometry; ' + THRUN + '; SapphireLib simulator, run by notebook/build/experiments.js; tests/localization/particle_filter_test.cpp.', T, key='drift')

_lcrit = [('Corrects anywhere on the field, including near walls', 5), ('Recovers from a big bump', 4),
          ('Copes when another robot blocks a sensor', 4), ('Usable on our 1 to 2 wall practice area', 3),
          ('Light on the brain\'s processor', 3), ('Few extra parts and ports', 2)]
_lscores = [[2, 1, 4, 5], [2, 4, 2, 5], [3, 2, 2, 5], [5, 2, 3, 3], [5, 5, 4, 2], [4, 2, 3, 3]]
page('Brainstorm and Pick Best Solution', 'Four Ways to Correct the Position', [
    TABLE([
        ['', 'A. Wall resets', 'B. GPS sensor', 'C. Kalman filter', 'D. Monte Carlo localization (MCL)'],
        ['How it works', 'At planned moments, press against or measure to a wall and overwrite x or y', 'A camera reads coded strips on the field walls and reports position', 'Blend odometry with distance sensor readings, weighted by trust in each', 'Keep hundreds of guesses of where the robot is; keep the ones the sensors agree with'],
        ['Strength', 'Simple and exact at that moment', 'Reports position directly', 'Cheap; well known', 'Many guesses at once; shrugs off a blocked sensor'],
        ['Weakness', 'Only fixes at the planned spots; costs time', 'Must see the strip, and must be a full tile from the walls', 'One guess only: a reading off another robot needs extra logic to reject', 'Heaviest on the processor; many settings'],
    ], widths=[1.0, 1.6, 1.6, 1.75, 1.95], size=9.5, bold_first_col=True),
    MATRIX(_lcrit, ['A. Wall reset', 'B. GPS', 'C. Kalman', 'D. MCL'], _lscores),
    H('Reasoning behind the key scores'),
    B([
        '**GPS scores 1 on the top criterion.** We considered it seriously. It can only localize while it can see the '
        'code strip, and it has to be a full tile away from the wall to work. Our robot scores and loads against the '
        'walls, exactly where the GPS goes blind.',
        '**Wall resets score 2 on bumps:** a bump is only fixed at the next planned reset, which may be after the miss.',
        '**Kalman scores 2 on blocked sensors:** it believes one position, so a distance reading off another robot '
        'drags that position unless we add separate rejection logic.',
        '**MCL scores 2 on processor load.** This is its real cost and the risk we take on (tested on page [[particles]]).',
    ]),
    BOX('**Decision (Cody Paschal): Monte Carlo localization on four distance sensors,** one facing each way. It is the '
        'only option that scores well on the three highest-weighted criteria. We also keep wall resets (A) as a '
        'one-line command for routines, because they are exact and cost nothing to offer.', fill='FFF2CC'),
], DELLAERT + '; ' + THRUN + '; ' + GPS + '; ' + DIST + '; R. Kalman, "A New Approach to Linear Filtering and Prediction Problems," 1960; 2654E Echo, MCL explanation video (YouTube).', T, key='locmatrix')

page('Research', 'How Monte Carlo Localization Works', [
    P('MCL comes from Dellaert, Fox, Burgard and Thrun (1999). The name is from the Monte Carlo casino: it works by '
      'making many random guesses. Each guess is called a **particle**: one possible position of the robot.'),
    IMGB('mcl_steps.png'),
    H('An everyday comparison'),
    P('You wake up in a dark room you know well. You are not sure exactly where you are standing. You reach out: a '
      'wall 2 ft to your left, a wall 6 ft ahead. Only a few spots in the room match both. MCL does the same with four '
      'distance sensors and a map of the field walls, 20 times a second.'),
    H('The sensor we use'),
    TABLE([
        ['V5 Distance Sensor', 'Value', 'What it means for the design'],
        ['Range', '20 mm to 2000 mm (about 78 in)', 'A reading past 78 in is ignored'],
        ['Accuracy', 'About 15 mm under 200 mm; about 5% beyond', 'Far readings are trusted less than near ones'],
        ['Update rate', 'About 30 times a second', 'No point running the localizer much faster'],
        ['Sees', 'The nearest thing in a cone, not a thin line', 'Caused a real bug later (page [[cone]])'],
    ], widths=[1.7, 2.8, 3.4]),
    H('Our design choices'),
    B([
        '**Particles hold x and y only.** Heading comes from the inertial sensor, which is far better at heading than '
        'distance sensors are. Fewer unknowns means fewer particles needed.',
        '**MCL advises, odometry drives.** MCL runs beside odometry and nudges odometry\'s position. Every motion still '
        'reads one position from one place, so nothing else in the library had to change.',
        '**An "outlier" allowance.** Each reading has a chance of being something the map cannot explain (another '
        'robot). Such a reading stops telling the particles apart instead of dragging them all to a wrong spot.',
        '**Repeatable randomness.** The random numbers come from a seed, so the same test gives the same particles '
        'every run, on the robot and on a laptop.',
    ]),
], DELLAERT + '; ' + THRUN + '; M. Arulampalam et al., "A Tutorial on Particle Filters," IEEE Trans. Signal Processing 50(2), 2002; ' + DIST + '; 2654E Echo, MCL explanation video (YouTube).', T, key='mclhow')

page('Programming', 'The Particle Filter Code', [
    P('These are the two core steps, trimmed. "Predict" moves every particle the way odometry says the robot moved. '
      '"Weigh" scores each particle by how well the walls it would see match what the sensors actually read.'),
    CODE(r'''
// PREDICT: move every particle by what odometry measured, plus a little random noise.
// The noise is what lets the cloud spread out to cover odometry's possible error.
void ParticleFilter::predict(double dxIn, double dyIn, double turnedDeg) {
    const double sigmaIn = noise.baseIn + noise.perInch * std::hypot(dxIn, dyIn)
                         + noise.perDegreeIn * std::fabs(turnedDeg);   // moved more -> less sure
    for (Particle& p : particles_) {
        p.xIn += dxIn + sigmaIn * rng_.fastGaussian();
        p.yIn += dyIn + sigmaIn * rng_.fastGaussian();
    }
}

// WEIGH: for each particle, for each sensor: "if the robot were HERE, how far away
// would this sensor see a wall?" Compare that with the real reading.
for (std::size_t i = 0; i < particles_.size(); ++i) {
    double logLikelihood = map_.contains(p.xIn, p.yIn) ? 0.0 : kOutsideLogPenalty;  // off the field?
    for (const std::size_t s : usedSensors_) {
        double expectedIn = casters_[s].castIn(p.xIn + ray.xIn, p.yIn + ray.yIn);  // distance to wall
        logLikelihood += scorers_[s].logLikelihood(expectedIn);                    // good match = high
    }
    logWeights_[i] += logLikelihood;   // added as logarithms: four tiny numbers multiplied
}                                      // together would round to zero
'''),
    H('Then: resample and estimate'),
    P('Particles with low weight are dropped and high-weight ones are copied, but only once the weights have become '
      'lopsided (fewer than half the particles doing useful work). The estimate is the weighted average.'),
    H('Two changes from the textbook, both found in the simulator'),
    TABLE([
        ['Textbook behaviour', 'What we saw', 'Our change'],
        ['Recovery (scattering fresh particles) triggers when the average match gets worse', 'It fired constantly: a far wall always matches worse than a near one, so just driving toward and away from walls looked like "getting worse"', 'Measure the match as a fraction of a perfect match for that reading'],
        ['Recovery runs whenever the match dips', 'Ordinary driving dips a little all the time', 'A trigger level: ordinary driving stays above 0.9, a 7 in shove drops it under 0.6'],
    ], widths=[2.3, 3.3, 2.3], size=10),
    BOX('**Desktop test results:** after 80 s of laps on deliberately bad odometry (4% wheel error, 1.5 degree heading '
        'bias, slow creep), odometry was 28.8 in off and MCL 0.13 in. After an 8 in bump, the filter found the robot '
        'again in 4 updates with recovery on, and never did with it off while standing still.'),
], THRUN + ' (sec. 8.3, augmented MCL); ' + DELLAERT + '; Arulampalam et al. 2002; D. Blackman and S. Vigna, xoshiro generators, prng.di.unimi.it; SapphireLib src/sapphirelib/localization/particle_filter.cpp.', T)

page('CAD/Design and Prototype', 'Prototype: the Simulator', [
    H('Why a simulator'),
    P('The competition robot was still being built, and MCL has dozens of settings. Trying them one download at a time '
      'on a robot that did not exist yet was not possible. We built the test bench first: a simulator that runs in a '
      'web browser.'),
    SIDE('sim_field.png', 3.5, [
        H('What it is'),
        B([
            'A top-down field with a robot that drives using **the library\'s own math**, copied line for line into '
            'JavaScript.',
            'The robot\'s drive is the model Auto-Tune measures.',
            'The "world" can be made worse on purpose: worn wheels, sensor noise, a bump, a defender robot.',
            'It draws the particles, the sensor beams, and both positions (true and believed).',
        ], 11.5),
    ], caption='The Field tour test route (true path).'),
    H('How we know the copy matches the robot code'),
    P('The JavaScript copy is held to the same test values as the C++: a filter run from the same random seed must '
      'land on the same particles to nine decimal places. If someone changes one side, the test fails until the other '
      'side matches.'),
    IMGB('drift_series.png'),
    H('First results'),
    TABLE([
        ['Same run as page [[drift]]', 'Odometry alone', 'Odometry + MCL'],
        ['Largest error', '%.1f in' % odom_peak, '%.1f in (at the bump)' % max(s['e'] for s in DRIFT['mcl'])],
        ['Error at the end', '%.1f in' % odom_end, '%.1f in' % mcl_end],
    ], widths=[3.1, 2.4, 2.4], fills={(1, 1): RED, (2, 1): RED, (1, 2): YELLOW, (2, 2): GREEN}),
    H('A bug only the simulator could show'),
    P('Before the routine tells odometry where the robot starts, odometry believes it is at (0, 0). A localizer allowed '
      'to correct at that moment briefly locked onto a spot more than 30 in away that happened to match the walls. '
      '**Change:** the localizer now waits until the start position has been set before it may correct anything.'),
    BOX('**Reflection:** the simulator is a prototype in the true sense. It let us try ideas in seconds and throw most '
        'of them away. Its limit is stated on its own front page: it shows what the robot will do only as far as the '
        'simulated world matches the real one. Every number from it in this notebook is labelled "simulated".'),
], DELLAERT + '; ' + THRUN + '; WPILib robot simulation, docs.wpilib.org; SapphireLib tools/sim/README.md and tools/sim/test/mcl.test.js.', T, key='simulator')

page('Test', 'MCL Against Odometry Alone', [
    H('Question'),
    P('Does MCL keep the position more accurate than odometry alone, in the conditions a match actually has?'),
    H('Procedure (repeatable)'),
    B([
        '**Routines:** Field tour (25 s), Square laps (24 s), Sprints (22 s).',
        '**Conditions:** Normal (wheels 1% off, 0.5% slip, IMU drift 0.5 deg/min). Worn wheels (3%, 2%, 3 deg/min). '
        'A 6 in bump at 6 s. A defender robot patrolling and blocking sensors.',
        '**Each combination run with correction off and on, for random seeds 1 to 5:** 120 runs.',
        '**Measured every 10 ms:** distance from the position the motions use to the true position. Reported: the mean '
        'over the run, averaged over the 5 seeds.',
    ]),
    IMGB('mcl_vs_odom.png'),
    TABLE([['Condition', 'Routine', 'Odometry alone: mean / end (in)', 'With MCL: mean / end (in)', 'Mean error cut by']] + [
        [r['condition'], r['routine'].replace('Field tour', 'Tour').replace('Square laps', 'Laps'),
         '%.2f / %.2f' % (r['odomMean'], r['odomEnd']), '%.2f / %.2f' % (r['mclMean'], r['mclEnd']),
         '%d%%' % round(100 * (1 - r['mclMean'] / r['odomMean']))] for r in MVO['rows']],
        widths=[1.4, 1.0, 2.2, 1.9, 1.4], size=9.5,
        fills={(i + 1, 4): (GREEN if 1 - r['mclMean'] / r['odomMean'] > 0.5 else YELLOW) for i, r in enumerate(MVO['rows'])}),
    H('Analysis'),
    B([
        'MCL lowered the mean error in **all 12** combinations, by %d%% to %d%%.' % (
            min(round(100 * (1 - r['mclMean'] / r['odomMean'])) for r in MVO['rows']),
            max(round(100 * (1 - r['mclMean'] / r['odomMean'])) for r in MVO['rows'])),
        'The biggest gain is after a bump: odometry stays about 5.6 in off for the rest of the routine; MCL averages '
        'under 0.75 in.',
        'The smallest gain is Laps with worn wheels. On a closed lap, odometry\'s errors partly cancel by the time it '
        'returns to the start, so there is less to fix. This is also why we report the mean, not only the end.',
        'A defender blocking sensors made almost no difference, which is the "outlier allowance" doing its job.',
    ]),
    BOX('**Decision (Cody Paschal):** MCL meets the under-1-in criterion in simulation in every condition, so it goes '
        'onto the robot. **Limit:** this is simulated data. The real sensors, walls and tiles still have to agree '
        '(page [[mclstatus]]).', fill='FFF2CC'),
], DELLAERT + '; ' + THRUN + '; SapphireLib simulator (tools/sim), run by notebook/build/experiments.js; tools/sim/test/sim.test.js.', T)

_pc = {r['particles']: r for r in PC['rows']}
page('Test', 'How Many Particles?', [
    H('Why this test'),
    P('The decision matrix flagged MCL\'s cost on the processor as its weak point. More particles should mean a better '
      'estimate but more work for the brain, which also has to run the drive, the lift and the logger.'),
    H('Test 1: accuracy against particle count (simulator)'),
    P('**Procedure:** Field tour, worn wheels, 6 in bump at 6 s, seeds 1 to 5. The particle count is the only thing '
      'changed: 50, 100, 300, 1000. Measured: mean error, and seconds after the bump until the error is back under 1 in.'),
    IMGB('particle_count.png'),
    H('Test 2: cost on the brain'),
    P('**Procedure:** the update was timed by counting processor instructions in an emulator of the brain\'s chip '
      '(ARM Cortex-A9), before and after we replaced slow math with lookup tables.'),
    TABLE([
        ['Particles', 'Time per update, first version', 'Time per update, with tables', 'Share of the brain at 20 updates/s'],
        ['300', '1.1 to 2.1 ms', '0.34 to 0.76 ms', '0.7 to 1.5%'],
        ['1000', '3.5 to 6.5 ms', '1.1 to 2.4 ms', '2.1 to 4.8%'],
        ['3000', '10 to 19 ms', '3.2 to 7.1 ms', '6 to 14%'],
    ], widths=[1.2, 2.3, 2.3, 2.1], fills={(1, 2): GREEN, (2, 2): GREEN}),
    H('What made it about 3 times faster'),
    B([
        'Random noise from a pre-computed table (the "ziggurat" method) instead of logarithms and square roots.',
        'The sensor score from a lookup table instead of an exponential per particle per sensor.',
        'All of one sensor\'s wall distances worked out together, with no division per particle.',
    ]),
    H('Analysis'),
    B([
        'From 100 to 1000 particles the mean error only moves from %.2f to %.2f in. Sensor noise, not particle count, '
        'sets the accuracy.' % (_pc[100]['mean'], _pc[1000]['mean']),
        'Recovery after a bump does improve: %.2f s at 50 particles, %.2f s at 1000.' % (_pc[50]['recoverS'], _pc[1000]['recoverS']),
        'With the tables, 1000 particles cost about what 300 did before.',
    ]),
    BOX('**Decision (Cody Paschal):** the library default stays at 300 particles, which is under 1.5% of the brain, '
        'and the count is a setting in TUNE.CFG so it can be raised for faster bump recovery without a rebuild. The '
        'processor risk from the decision matrix is closed.', fill='FFF2CC'),
], 'G. Marsaglia and W. Tsang, "The Ziggurat Method for Generating Random Variables," J. Statistical Software 5(8), 2000, jstatsoft.org; QEMU emulator, qemu.org; SapphireLib docs/TOOLS.md and simulator.', T, key='particles')

page('Programming', 'Applying a Correction Without Jerking the Robot', [
    H('The problem'),
    P('When MCL decides odometry is 2 in off, the obvious thing is to move odometry\'s position by 2 in. But a motion '
      'is running at that moment, and its PID is watching that position. A sudden 2 in jump looks to the D term like '
      'the robot moving at enormous speed.'),
    SIDE('correction_rate.png', 3.7, [
        P('**Calculation:** an Auto-Tuned drive has a D gain of about 0.7 V per in/s. A jump of only 0.2 in inside one '
          '10 ms loop looks like 20 in/s, which is a 14 V kick: more than a motor\'s whole range.', 11.5),
        P('**Our answer:** corrections "ease in". Odometry\'s position slides toward MCL\'s at a capped speed.', 11.5),
    ]),
    CODE(r'''
// each 10 ms, move the position toward the correction, but no more than maxStepIn
PoseDelta correctionStep(double remainingXIn, double remainingYIn, double maxStepIn) {
    const double remainingIn = std::hypot(remainingXIn, remainingYIn);
    if (remainingIn <= maxStepIn) return {remainingXIn, remainingYIn};   // close: finish it
    const double scale = maxStepIn / remainingIn;                        // far: one capped step
    return {remainingXIn * scale, remainingYIn * scale};
}
'''),
    H('When MCL is allowed to correct at all'),
    TABLE([
        ['Rule', 'Why'],
        ['The particle cloud must be tight (within 3 in)', 'A spread-out cloud means MCL is not sure'],
        ['At least two sensors must agree with the map', 'One sensor alone can be fooled by a robot or game piece'],
        ['Not while spinning fast', 'Readings are smeared and slightly old during a fast turn'],
        ['Only after the start position is set', 'The simulator bug on page [[simulator]]'],
    ], widths=[3.7, 4.2]),
    H('Iteration: the cap changed after a real run'),
    TABLE([
        ['Version', 'Cap', 'Evidence', 'Result'],
        ['Library default', '4 in/s', 'Calculation above: at most a 2.8 V nudge', 'Smooth, but slow to apply a big correction'],
        ['Our robot now', '18 in/s', 'Robot log SL000021: odometry ran 10 in behind the localizer on a drive to the goal, and the robot hit the goal at full power still thinking it was short', 'Correction keeps up on a fast approach'],
    ], widths=[1.4, 0.9, 3.5, 2.1], size=10, fills={(2, 1): GREEN}),
    BOX('**Traceable change:** the 4 in/s cap came from a calculation; the move to 18 in/s on our robot came from a '
        'logged run where the calculation\'s answer was too cautious. The library keeps 4 as its default and each robot '
        'sets its own. This trade-off (smooth against quick) is still being tuned.'),
], THRUN + '; ' + ASTROM + ', ch. 10 (derivative action); SapphireLib src/sapphirelib/odom/odometry_math.cpp and src/robot/tune.cpp.', T)

page('CAD/Design and Prototype', 'Finding Where the Sensors Really Are', [
    H('The problem'),
    P('MCL compares each sensor\'s reading with the map. To do that it must know exactly where each sensor sits on the '
      'robot. Any error there goes straight into the position. With four sensors tucked inside the chassis, a ruler is '
      'not accurate enough, the same lesson as the tracking wheel offsets.'),
    H('Three versions, each from what the last one could not do'),
    TABLE([
        ['Version', 'How it works', 'Tested accuracy (simulated)', 'What sent us to the next version'],
        ['1. Full field', 'Robot spins one turn each way at the middle of a field; a fit finds the start position and every sensor\'s offset against the four walls', 'Within about 0.1 in along each beam', 'We do not have a full field. It could never be run at our practice area.'],
        ['2. One wall or a corner', 'Spin near a wall or in a corner. The sensors\' spacing is held from the configured values; everything else is found', 'Within about 0.05 in of where the spacing puts it', 'The spacing itself is still taken from a tape measure'],
        ['3. A box of known size', 'Spin inside a box (or between two walls) whose inside size is known, which pins the spacing as well', 'Every sensor within 0.11 in; about 0.03 in typical in a 40 in box', 'Current version'],
    ], widths=[1.3, 2.7, 1.8, 2.1], size=9.5),
    H('The fit, in plain words'),
    P('The robot collects hundreds of readings while it spins. The program then asks: "what sensor positions would '
      'make all of these readings line up with straight walls?" It adjusts the positions a little at a time until they '
      'fit best (the Levenberg-Marquardt method). Readings that are far from any wall, such as a chair or a person '
      'walking past, are given no say at all (Tukey\'s biweight).'),
    H('What testing in random rooms taught us'),
    TABLE([
        ['Finding from the stress tests', 'Change'],
        ['Trusting each reading by its **measured** distance pushed every offset out by about 0.08 in', 'Trust is based on the predicted distance'],
        ['A gentler outlier rule let furniture drag the answer into a compromise', 'The strict rule from the first pass'],
        ['From one wall, the fit\'s other walls settled on a cabinet\'s face', 'A wall more than 3 degrees out of square is left out'],
        ['A flat object across the room passed for a field wall and moved every sensor 4 in', 'The spacing may change by 3 in at most'],
        ['A 20 in box cannot work: the robot cannot spin in it', 'Recommended box is 34 to 56 in'],
    ], widths=[4.6, 3.3], size=10),
    BOX('**Reflection:** version 1 was correct and useless to us. We had designed for the field in the game manual, not '
        'for the room we actually practise in. That constraint is on page G-1; we had not applied it. Versions 2 and 3 '
        'exist because we went back to it. Decided by Cody Paschal.', fill='FFF2CC'),
], 'D. Marquardt, "An Algorithm for Least-Squares Estimation of Nonlinear Parameters," J. SIAM 11(2), 1963; A. Beaton and J. Tukey, "The Fitting of Power Series," Technometrics 16(2), 1974; ' + DIST + '; SapphireLib docs/LOCALIZATION.md.', T)

# ------------------------------------------------------------------ competition robot

page('Robot Iterations', 'Moving to the Competition Robot', [
    H('What changed'),
    P('Until now SapphireLib ran on the test chassis and in the simulator. With the competition robot built (Section '
      'H), the whole program was moved onto it. This is where the layered design from page [[design]] was tested for '
      'real: only the top layer should need to change.'),
    TABLE([
        ['Part of the program', 'Changed for the new robot?', 'What changed'],
        ['Library (drive, odometry, motions, MCL, tools)', 'No', 'Nothing rewritten. New features were added beside it (orbit mode, Home tab buttons)'],
        ['Ports', 'Yes', 'One file, config.hpp: every port written once'],
        ['Devices', 'Yes', 'Three-motor lift, pneumatic claw, extender, side piston'],
        ['Macros', 'Yes', 'New controls for the new mechanisms'],
        ['Tuning', 'Re-run', 'Auto-Tune run again; offsets and sensor mounts re-calibrated'],
    ], widths=[3.3, 1.9, 2.7], fills={(1, 1): GREEN}),
    H('A new-chassis checklist'),
    P('Because every calibration depends on the one before it, we wrote the order down so a rebuild is repeatable:'),
    B([
        '1. Inertial sensor scale (spin a known number of turns).',
        '2. Tracking wheel diameters, then their offsets (spin in place).',
        '3. Auto-Tune each axis, then the lift.',
        '4. Distance sensor mounts (spin in the box).',
        '5. Localizer settings, from the robot\'s own logs.',
    ]),
    SIDE('orbit.png', 3.0, [
        H('New driver mode: orbit'),
        P('With a drive that can slide sideways we added a mode for defending a goal. The robot locks onto a pivot '
          'point (the goal) and the stick moves it **around** that point while it keeps facing in.', 11.5),
        P('What makes ours different from circling by hand: the robot strafes along the true circle while the center '
          'wheels push it toward the goal, so it stays pinned against it and is hard to shove off.', 11.5),
    ]),
    BOX('**Reflection:** the port changed the robot\'s own files and rewrote no library file. That is the first real evidence '
        'for the decision on page G-6 and the layers on page [[design]]: code written for one specific robot would '
        'have been rewritten here.'),
], PROS + ' (device configuration); ' + LEMLIB + ' (chassis configuration); 96671H notebook, Section H; SapphireLib docs/NEW_CHASSIS.md.', V)

page('Programming', 'Trajectories: Planning the Whole Drive', [
    H('What testing showed'),
    P('moveToPoint aims at the target and lets the PID push. On a diagonal, a holonomic drive\'s forward and sideways '
      'directions are not equally strong, so the robot does not travel in a straight line: it bends.'),
    SIDE('diagonal.png', 3.8, [
        P('**Procedure (simulator):** drive from (-48, -48) to (24, 0), an 86 in diagonal, with perfect sensors. Once '
          'with moveToPoint, once with a planned trajectory. Measured every 10 ms: distance from the straight line.', 11.5),
        TABLE([
            ['', 'moveToPoint', 'Trajectory'],
            ['Worst off the line', '%.2f in' % DIAG['moveToPoint']['worstIn'], '%.2f in' % DIAG['trajectory']['worstIn']],
            ['Mean off the line', '%.2f in' % DIAG['moveToPoint']['meanIn'], '%.2f in' % DIAG['trajectory']['meanIn']],
            ['End error', '%.2f in' % DIAG['moveToPoint']['endErrorIn'], '%.2f in' % DIAG['trajectory']['endErrorIn']],
            ['Time', '%.2f s' % DIAG['moveToPoint']['timeS'], '%.2f s' % DIAG['trajectory']['timeS']],
        ], widths=[1.6, 1.2, 1.1], size=10, fills={(1, 1): RED, (1, 2): GREEN, (4, 1): GREEN, (4, 2): YELLOW}),
    ], img_left=True),
    H('How a trajectory is different'),
    P('A trajectory is planned **before** the robot moves: where it should be, how fast, and facing which way, at every '
      'instant. The robot then follows that plan.'),
    B([
        '**The path** is a smooth curve through the waypoints (a cubic spline), so there are no sudden corners.',
        '**The speed** is the fastest the measured motor model allows. The volts needed for forward, sideways and '
        'turning must add up to no more than the voltage budget, because that sum is what the hardest-working wheel '
        'sees. This is what slows the plan on diagonals and in turns, automatically.',
        '**Following** uses feedforward (the volts the model says this speed needs) plus a small PID on each axis to '
        'remove what is left.',
        '**If the robot is blocked,** the plan pauses and waits for it, instead of running ahead.',
    ]),
    H('Trade-off'),
    P('The trajectory is %.1f s slower on this drive (%.2f s against %.2f s), because it will not ask the wheels for '
      'more than they can give. In return it is %d times closer to the line. For driving between field elements with a '
      'few inches to spare, we accept the time.'
      % (DIAG['trajectory']['timeS'] - DIAG['moveToPoint']['timeS'], DIAG['trajectory']['timeS'], DIAG['moveToPoint']['timeS'],
         round(DIAG['moveToPoint']['worstIn'] / DIAG['trajectory']['worstIn']))),
    BOX('**Decision (Cody Paschal):** keep both. moveToPoint for short, simple moves; trajectories for long or curved '
        'routes and anywhere the route passes close to something. Trajectories can also branch part-way on a sensor '
        'reading without stopping. **Status:** trajectories have been driven on the robot.', fill='FFF2CC'),
], 'WPILib docs, Trajectory Generation and Following, docs.wpilib.org; ' + COULTER + '; ' + ASTROM + ' (feedforward design); SapphireLib docs/TRAJECTORIES.md; simulator, run by notebook/build/experiments.js.', V)

_acrit = [('We can trust what it will do (readable)', 5), ('Quick to adjust at a tournament', 5),
          ('Mechanisms timed to the drive', 4), ('Fast to make the first routine', 3), ('Little library code to maintain', 2)]
_ascores = [[1, 4, 5], [1, 3, 5], [4, 2, 5], [5, 2, 4], [1, 5, 4]]
page('Brainstorm and Pick Best Solution', 'How Should We Write Autonomous Routines?', [
    H('The problem'),
    P('The library can now drive accurately. The question became how a person tells it what to do. We built one '
      'approach, used a second, and replaced both with a third.'),
    TABLE([
        ['', 'A. Teach mode', 'B. Library calls directly', 'C. Short auton functions'],
        ['What it is', 'Drive the route once by hand while the robot records position and buttons; replay it as the autonomous', 'Write each motion with the drivetrain\'s full commands and settings', 'One short word per motion, with sensible defaults: drive(24); turnTo(90);'],
        ['Good', 'No code to write; fast first result', 'Total control', 'Reads like the plan on the whiteboard; a number is easy to change'],
        ['Bad', 'The routine is a recording, not text. To change one stop you re-drive the whole thing. No way to read what it will do', 'Long lines; easy to mis-set an option; slow to edit between matches', 'A layer of our own to maintain'],
    ], widths=[1.0, 2.4, 2.1, 2.4], size=10, bold_first_col=True),
    MATRIX(_acrit, ['A. Teach mode', 'B. Direct calls', 'C. Auton functions'], _ascores),
    H('What happened with teach mode'),
    P('We built teach mode fully: recording, smoothing the recorded drive into a trajectory, and replaying the button '
      'presses on the plan\'s clock. It had unit tests and it worked in them. But it added a large amount of code that '
      'was hard to debug, and a recorded routine cannot be read or edited by hand: when it did the wrong thing there '
      'was nothing to look at except a file of numbers.'),
    BOX('**Decision (Cody Paschal): remove teach mode and write routines with short auton functions (C).** Teach mode '
        'was too complex to trust. An autonomous we cannot read is an autonomous we cannot fix in the pits. The style '
        'of C follows RW-Template\'s workflow, which we researched as an example of routines that stay short.',
        fill='FFF2CC'),
    P('**Reflection:** this is the one place in this section where finished, tested code was deleted. The tests proved '
      'teach mode did what we designed. They could not tell us the design was wrong for how we work at a tournament. '
      'The first two criteria in the matrix were added **after** using it, and they are now the first thing we ask of '
      'any tool: can a person read it and change it quickly?'),
], 'RW-Template (GitHub), autonomous workflow; ' + EZ + ' (autonomous routines); ' + LEMLIB + '; SapphireLib docs/QUICKSTART.md and docs/ROADMAP.md.', V)

page('Programming', 'Auton Functions', [
    P('This is part of a real routine from our robot: stack three Pins on a Goal. Each line is one action. Compare it '
      'with the whiteboard plan in the comments.'),
    CODE(r'''
void triplePinPoints() {
    setPose(-60.6, -5, 90);                       // tell the robot where it starts
    macros::setLiftLevel(macros::Goal::alliance, 1);

    moveToPoint(-50, -5);                         // 1: straight out from the wall
    if (!ontoGoal(kGoalXIn, kGoalYIn)) return;    //    onto the goal; stop the routine if it failed
    scoreOnGoal();                                //    preload scored at level 1

    offGoalTo(-50.8, 0);                          // 2: back off the goal
    if (!ontoPin(-23.55, -23.55)) return;         //    second Pin; missed it? stop here
    macros::setLiftLevel(macros::Goal::alliance, 2);
    backTo(-29.8, -18.1);
    if (!ontoGoal(kGoalXIn, kGoalYIn)) return;
    scoreOnGoal();                                //    scored at level 2
    ...
}
'''),
    H('What each function gives a routine'),
    TABLE([
        ['Feature', 'Example', 'Why we needed it'],
        ['Speed cap', 'drive(24, {.maxVolts = 6})', 'Creep the last inches into a goal'],
        ['Chaining', 'moveToPoint(x, y, {.chain = true})', 'Flow into the next motion without stopping'],
        ['Background motion', 'runAsync(...); waitUntilNear(x, y, 6);', 'Raise the lift while still driving'],
        ['Cancel', 'cancelMotion()', 'Stop the drive the moment the claw closes'],
        ['Forced turn direction', 'turnTo(180, clockwise)', 'Keep a back corner from swinging into a wall'],
        ['Wall reset', 'resetBack()', 'Option A from page [[locmatrix]], as one line'],
    ], widths=[1.9, 3.0, 3.0]),
    H('Design rule: stop rather than guess'),
    P('Every step that can fail returns true or false, and the routine checks it (`if (!ontoPin(...)) return;`). If '
      'the robot misses the second Pin, it stops there instead of driving to the goal and "scoring" nothing, or '
      'scoring in the wrong place. A stopped robot in a known position is easier for the driver to recover than one '
      'that carried on.'),
    H('Tools that go with it'),
    B([
        'The simulator runs the same routine with the same function names, so a route can be tried before the robot '
        'is free.',
        'The simulator\'s Paths tab lets us draw a route on the field and writes the C++ for it.',
        'The auton selector remembers the last choice on the SD card and can be changed from the controller while '
        'disabled.',
    ]),
    BOX('**Status:** routines written this way are what the robot runs today. In the simulator the Triple Pin routine '
        'finishes inside 14.5 s with each Pin within 0.75 in of the goal\'s middle.'),
], 'RW-Template (GitHub); ' + EZ + '; ' + LEMLIB + ' (async motions, waitUntil); SapphireLib docs/QUICKSTART.md, src/robot/autons.cpp and tools/sim/test/sim.test.js.', V)

page('Test', 'First Runs on the Robot: What the Logs Showed', [
    H('The test'),
    P('The "auto cycle" is a driver-assist: one button and the robot shuttles between the loader and the goal by '
      'itself while the driver works the lift and claw. It was the first routine run repeatedly on the competition '
      'robot with the SD logger recording.'),
    IMGB('flow_autocycle.png', caption='One auto cycle. Touching a drive stick cancels it at any point.'),
    H('Procedure'),
    P('Run the cycle from the right-hand loader. After each run, pull the SD card and open the log in the analyzer. '
      'Each row below is one finding, the log it came from, and the change it led to.'),
    TABLE([
        ['Log', 'What the log showed', 'Cause', 'Change'],
        ['SL000028', 'Turns stopped 5.5 degrees short at 1.5 V. Moves stopped 1 in off sideways at 2.2 V. Each then waited out its 3 s timeout', 'Close to the target the PID asks for less voltage than it takes to start the robot moving', 'A minimum push per direction until first inside the threshold: 1 V forward, 4 V strafe, 2.5 V turn'],
        ['SL000029', 'Strafing at a 3 V minimum only crept', 'The center wheels drag sideways', 'Strafe minimum raised to 4 V'],
        ['SL000030', 'Creeping into the goal, the robot twisted 6 degrees and stayed there', '1.5 V was too little to turn back', 'The minimum returns if the error grows past twice the threshold'],
        ['SL000028', 'The drive braked about twice as hard as designed from 17 in out', 'Tracking wheel readings changed only every other loop, so the D term read double, then zero', 'Rotation sensors set to report every 5 ms'],
        ['SL000028', 'Localizer moved the robot 3.7 in toward the wall at the loader', 'The front sensor saw the loader\'s face; the map only had walls', 'The four loaders added to the map'],
    ], widths=[0.9, 2.6, 2.3, 2.1], size=9.5),
    H('What this shows'),
    B([
        '**None of these appeared in the simulator.** The simulated robot has no sticky friction near zero speed, '
        'sensors that update every loop, and no loaders. Each one is a way the real world differs from our model.',
        '**All five were found in the first three logged runs.** Without the logger, "the robot stops a bit short '
        'sometimes" would have been the whole diagnosis.',
        '**Each fix went back into the simulator too,** so the model is now closer to the robot than it was.',
    ]),
    BOX('**Reflection:** this page is why telemetry was built before the robot port. The plan had logging as a '
        '"nice to have" tool. It turned out to be how every later decision was made. Decided by Cody Paschal from the '
        'logs listed.', fill='FFF2CC'),
], PROS + ' (Rotation sensor data rate); ' + ASTROM + ' (static friction, dead zones); SapphireLib docs/ROADMAP.md (auto cycle notes, logs SL000028 to SL000030) and src/robot/autocycle.cpp.', V)

page('Test', 'The Sensor Sees a Cone, Not a Line', [
    H('What the log showed'),
    P('After the loaders were added to the map, log SL000035 showed a new miss: approaching a loader, the localizer '
      'moved the robot\'s position and the robot stopped **3.3 in short**.'),
    SIDE('sensor_cone.png', 3.6, [
        H('Cause'),
        P('Our map math treated each distance sensor as a thin line: "what is straight ahead?" The real sensor reads '
          'the nearest thing anywhere inside a cone. Beside a loader, the center line passed the loader and hit the '
          'wall behind it. The sensor itself saw the loader\'s corner, 3.7 in nearer. The localizer trusted its line '
          'model, decided the reading meant the robot was 3.7 in closer to the wall, and "corrected" a position that '
          'had been right.', 11.5),
    ]),
    H('How we measured the cone'),
    P('Three logs contained approaches to the loaders. For each reading we knew where the robot was and what the sensor '
      'reported, so we could work out how far off-center the loader was when the sensor first picked it up. The '
      'readings fit a half-angle of about 15 degrees.'),
    H('The change'),
    B([
        'Walls are still read along the center line (a flat wall reads the same across the cone).',
        'Anything **added** to the map (loaders, goals) is checked across the whole 15 degree cone, and the nearest '
        'hit is used.',
        'Faces turned more than 60 degrees away from the beam are left out: the sensor does not get a return from them.',
    ]),
    TABLE([
        ['Test', 'Line model', 'Cone model'],
        ['Replay of the SL000035 approach (desktop test)', 'Picks the spot the localizer wrongly went to', 'Picks where the robot really was'],
        ['Cone against many thin rays 0.005 degrees apart', '-', 'Matches'],
        ['A map with walls only (all earlier tests)', 'Same result', 'Same result'],
    ], widths=[3.5, 2.3, 2.1], fills={(1, 1): RED, (1, 2): GREEN}),
    BOX('**Reflection:** the cone was in our own research table (page [[mclhow]], last row) weeks earlier, and we still '
        'modelled a line because it was simpler. The fix was small. Finding it needed a log, a replay and a diagram. '
        'We now check each simplification in the model against the sensor\'s real behaviour before trusting a '
        'correction near field elements. **Status:** in the library and tested on the desktop from the real log. '
        'Decided by Cody Paschal.', fill='FFF2CC'),
], DIST + '; ' + THRUN + ' (sec. 6.3, beam models of range finders); SapphireLib tests/localization/field_map_test.cpp and particle_filter_test.cpp; robot log SL000035.', V, key='cone')

_rules = list(CLAW['rules'].items())


def caught_at(rule_index, speed):
    return next(r for r in _rules[rule_index][1] if r['speed'] == speed)['caught'] * 100


page('Test', 'autoClaw: Closing at the Right Moment', [
    H('The problem'),
    P('autoClaw closes the claw by itself when a distance sensor inside it sees a Pin arrive. The first version closed '
      'when the reading dropped to a fixed 125 mm. In practice it grabbed well at one approach speed and missed at '
      'others.'),
    H('Why a fixed distance cannot work'),
    P('Between the sensor seeing the Pin and the jaws being shut, time passes: the sensor\'s reading is already about '
      '15 ms old, the program looks once per loop, and the air valve takes about 60 ms to close the jaws. During that '
      'time the robot keeps moving. Faster robot, more distance covered, so the Pin is deeper in the claw than planned.'),
    CODE(r'''
// how far will the Pin travel before the jaws are shut?  speed x (valve + loop + reading's age)
const double aheadMs = kClawLeadMs + tickMs + readingAgeMs;
aim.aimMm = readingMm - approachMmPerMs * aheadMs;   // where the Pin WILL be
aim.fires = aim.aimMm <= kGrabMm;                    // close now if that is the grab point
'''),
    H('Test (simulator)'),
    P('**Procedure:** the claw timing model: a sensor that measures every 33 ms and reports 15 ms late with noise, a '
      '20 ms program loop, a valve with a little variation. 400 grabs at each speed from 4 to 60 in/s for each of three '
      'rules. A grab is "caught" if the jaws shut with the Pin inside the grab window.'),
    IMGB('claw.png'),
    TABLE([['Rule', 'Slow (8 in/s)', 'Medium (28 in/s)', 'Fast (52 in/s)']] + [
        [name, '%d%%' % caught_at(i, 8), '%d%%' % caught_at(i, 28), '%d%%' % caught_at(i, 52)] for i, (name, _) in enumerate(_rules)],
        widths=[3.4, 1.5, 1.5, 1.5],
        fills={(1, 1): RED, (1, 2): GREEN, (1, 3): RED, (2, 1): GREEN, (2, 2): GREEN, (2, 3): YELLOW, (3, 1): GREEN, (3, 2): GREEN, (3, 3): GREEN}),
    H('Analysis'),
    P('The fixed trigger only works in a band around one speed: too early when slow, too late when fast. Leading by '
      'speed fixes both ends. Adding the reading\'s age and the loop time holds the catch rate a little higher at the '
      'top speeds, because a reading that has not changed for a while is older than it looks.'),
    BOX('**Decision (Cody Paschal): autoClaw leads by speed, reading age and loop time.** The robot also shows each '
        'grab on the controller and logs it. **Honest status:** the two constants on the robot (grab point 35 mm, lead '
        '125 ms) are starting values that have not yet been measured from logged grabs. That is the next test.',
        fill='FFF2CC'),
], DIST + '; ' + PROS + ' (ADI pneumatics, distance sensor); SapphireLib tools/sim/js/clawtiming.js, tools/sim/claw.html and src/robot/macros.cpp; run by notebook/build/experiments.js.', V)

page('Test', 'Where Localization Stands, and What Comes Next', [
    H('Honest status'),
    P('MCL is on the competition robot with its four sensors mounted and calibrated, and it corrects the position. '
      '**It is not yet good enough to use in a competition.** On the robot the corrected position still drifts more '
      'than in the simulator, and a wrong correction is worse than no correction. Our competition routines currently '
      'turn it off and run on odometry alone.'),
    TABLE([
        ['Feature', 'Simulated', 'On the robot', 'Trusted for competition'],
        ['Odometry and pose motions', 'Yes', 'Yes', 'Yes'],
        ['Auto-Tune gains', 'Yes', 'Yes', 'Yes'],
        ['Trajectories', 'Yes', 'Yes', 'Yes'],
        ['SD logging and the analyzer', 'Yes', 'Yes', 'Yes'],
        ['Monte Carlo localization', 'Yes', 'Works, drifts too much', 'Not yet'],
        ['Thermal compensation', 'Desktop math only', 'Not yet verified', 'Not yet'],
    ], widths=[3.0, 1.6, 1.9, 1.4],
        fills={(1, 3): GREEN, (2, 3): GREEN, (3, 3): GREEN, (4, 3): GREEN, (5, 2): YELLOW, (5, 3): RED, (6, 2): YELLOW, (6, 3): RED}),
    H('Why the gap between simulator and robot'),
    P('The simulator\'s world is cleaner than the real one. The auto cycle and cone pages each found a difference '
      '(sticky friction, the loaders, the cone). Each fix closed part of the gap. What remains is tuning the '
      'localizer\'s settings to the real sensors and tiles.'),
    H('Research for the next version (simulator, 10/27)'),
    P('**Procedure:** each idea was added to the simulator\'s localizer and compared with today\'s version on the same '
      'recorded paths: 12 random seeds, 10 scenarios (the standard set plus big bumps, worn wheels with bumps, '
      'mis-measured sensor mounts, a bad inertial sensor, and short wheels).'),
    IMGB('research_next.png'),
    B([
        '**Wheel-bias particles:** each particle also guesses how far off the tracking wheels are, so the filter learns '
        'the wheel error instead of fighting it forever. On worn wheels alone the gain was 45%.',
        '**Sensor resetting:** after a bump, place new particles where the wall readings say the robot is. Recovery in '
        '0.15 to 0.25 s, against 0.5 s.',
        '**Faster correction when far off:** no larger voltage kicks than today in the simulated closed loop.',
        '**Did not help:** more particles (as page [[particles]] predicted), and lower motion noise.',
    ]),
    BOX('**Plan:** (1) tune today\'s settings from the robot\'s logs using the simulator\'s Tune tab; (2) add wheel-bias '
        'particles, the largest single gain; (3) only then turn corrections on in a competition routine, and compare '
        'logged runs with it off and on. Decided by Cody Paschal.', fill='FFF2CC'),
], THRUN + ' (sec. 8.3); S. Lenser and M. Veloso, "Sensor Resetting Localization for Poorly Modelled Mobile Robots," IEEE ICRA 2000; ' + DELLAERT + '; SapphireLib simulator experiments of 10/27/26 and docs/LOCALIZATION.md.', V, key='mclstatus')

page('Team Management', 'Did SapphireLib Solve the Problem?', [
    P('We end this section by going back to the criteria and constraints on page G-1 and checking each against '
      'evidence in this section.'),
    TABLE([
        ['Criterion (page G-1)', 'Met?', 'Evidence'],
        ['Drive our robot (tank and holonomic, asterisk)', 'Yes', 'Six-motor mixer; all six motors used in turns'],
        ['Know where it is', 'Partly', 'Odometry: yes. Correction during a match: simulated under 1 in, not yet reliable on the robot'],
        ['Go where it is told', 'Yes', 'Point, pose, path and trajectory motions; diagonal within 0.3 in in simulation'],
        ['Tune itself', 'Yes', 'Model-based Auto-Tune: no overshoot at 6, 24 and 48 in; robot runs its gains'],
        ['Explain itself', 'Yes', 'SD logs; five problems found from the first three logged runs'],
    ], widths=[3.0, 0.8, 4.1],
        fills={(1, 1): GREEN, (2, 1): YELLOW, (3, 1): GREEN, (4, 1): GREEN, (5, 1): GREEN}),
    TABLE([
        ['Constraint', 'Held?', 'Note'],
        ['Runs on the V5 Brain with PROS 4.2.2', 'Yes', 'Localizer under 1.5% of the processor'],
        ['One programmer', 'Yes', 'Possible only because of desktop tests and the simulator'],
        ['One or two field walls', 'Yes, after a miss', 'Sensor calibration version 1 needed a full field; versions 2 and 3 do not'],
        ['Lasts two seasons', 'So far', 'Nothing game-specific in the library; the robot port changed no library file'],
        ['All code is ours to explain', 'Yes', 'This section'],
    ], widths=[3.0, 1.3, 3.6]),
    H('What we would do differently'),
    B([
        '**Build the logger first.** It was phase 4. Everything after it went faster and every decision after it has '
        'a log number beside it.',
        '**Put real hardware in the loop sooner.** The simulator found the math bugs. The robot found the physics we '
        'had left out: sticky friction, sensor update rates, the sensor\'s cone.',
        '**Decide what "easy to use at a tournament" means before building a tool.** Teach mode passed every test and '
        'was still the wrong tool.',
        '**Plan time for testing, not just writing.**',
    ]),
    H('What worked'),
    B([
        '**Prototyping in threes.** The autotuner and the sensor calibration each took three versions, and in both the '
        'third was only possible because of what the first two showed.',
        '**Decision matrices before code.** Four of them in this section. The one for the library itself is the reason '
        'the robot port took days.',
        '**Writing down limits.** Each page says whether its data is simulated, from a desktop test, or from the robot.',
    ]),
    BOX('**Next:** SapphireLib development continues in the monthly programming entries. The open items are on page '
        '[[mclstatus]].'),
], 'REC Foundation Engineering Notebook Rubric; 96671H notebook pages G-1 through this page; SapphireLib docs/ROADMAP.md.', V)

page('Monthly Gantt Charts', 'SapphireLib Final Chart and Analysis', [
    IMGB('gantt_final.png'),
    H('Original goal'),
    P('SapphireLib driving the competition robot in autonomous by mid-September.'),
    H('Accomplished'),
    B([
        'All five planned phases: chassis control, odometry, motions, tools, mechanisms.',
        'The library runs the competition robot in driver control and autonomous.',
        'Beyond the plan: trajectories, the simulator, the telemetry analyzer, TUNE.CFG.',
    ]),
    H('Partially complete'),
    B(['Monte Carlo localization: built, simulated and on the robot, but not yet accurate enough to use in matches.']),
    H('Pending'),
    B(['Measuring autoClaw\'s two constants from logs. Verifying thermal compensation in a long practice.']),
    H('Analysis and reasoning'),
    P('The goal was met about a month late: the competition robot ran on SapphireLib in mid-October, not mid-September. '
      'Comparing the hatched plan with the solid bars shows where the time went. Every phase ran long, and one bar '
      '(localization) was not in the plan at all. The causes, in order of size: three autotuner prototypes instead of '
      'one; localization, which started as an "extra goal" and became necessary once the bump test showed odometry '
      'alone would miss our accuracy criterion; and teach mode, which was built and then removed.'),
    P('The plan\'s mistake was treating each phase as "write it". Every phase was really "write it, test it, find what '
      'is wrong, change it", and the last three took longer than the first. Future plans will give testing its own bar.'),
    H('Future plans'),
    P('Tune the localizer from logs, add wheel-bias particles, and re-test (page [[mclstatus]]). These go on the '
      'team\'s monthly Gantt chart under A1 Programming from here on.'),
], '96671H notebook, page A-4 (How to Read Gantt Charts); SapphireLib docs/ROADMAP.md.', V, key='gantt_final')
