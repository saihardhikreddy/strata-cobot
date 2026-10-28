// Builds docs/Strata_Cobot_Presentation.pptx with pptxgenjs.
// Run from docs/:  node build_deck.js
const path = require('path');
const pptxgen = require('pptxgenjs');

const IMG = (f) => path.join(__dirname, 'images', f);
const OUT = path.join(__dirname, 'Strata_Cobot_Presentation.pptx');

// Mars-rust palette; the three ore colours are reused for azurite, malachite, vanadinite.
const THEME = {
  name: 'Strata Cobot',
  headFontFace: 'Calibri',
  bodyFontFace: 'Calibri',
  colors: {
    dk1: '1E2226', lt1: 'FFFFFF', dk2: '4A3B35', lt2: 'F1ECE8',
    accent1: 'C4501F', accent2: '2F6FB3', accent3: '3E9B4F', accent4: 'E07B1F',
    accent5: '6B7280', accent6: '8C3A15', hlink: '2F6FB3', folHlink: '6B4FA0',
  },
};
const HEX = THEME.colors;

const pres = new pptxgen();
pres.layout = 'LAYOUT_WIDE';            // 13.333 x 7.5 in
pres.author = 'C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy';
pres.title = 'Strata Cobot: autonomous ore sorting and haulage in ROS 2';
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;

// ---------- layouts ----------
pres.defineSlideMaster({
  title: 'TITLE_DARK',
  background: { color: HEX.dk1 },
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 0.7, y: 2.0, w: 6.4, h: 1.6, fontSize: 48, bold: true, color: C.background1, valign: 'bottom', margin: 0 }, text: 'Title' } },
    { placeholder: { options: { name: 'body', type: 'body', x: 0.7, y: 3.75, w: 6.4, h: 1.3, fontSize: 20, color: C.background2, valign: 'top', margin: 0 }, text: 'Subtitle' } },
  ],
});
pres.defineSlideMaster({
  title: 'CONTENT',
  background: { color: HEX.lt1 },
  margin: [0.5, 0.6, 0.6, 0.6],
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 0.6, y: 0.35, w: 12.1, h: 0.9, fontSize: 32, bold: true, color: C.text1, valign: 'middle', margin: 0 }, text: 'Title' } },
  ],
  slideNumber: { x: 12.3, y: 6.95, w: 0.5, h: 0.3, fontSize: 10, color: HEX.accent5, align: 'right' },
});
pres.defineSlideMaster({
  title: 'CLOSING_DARK',
  background: { color: HEX.dk1 },
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 0.7, y: 0.6, w: 11.9, h: 1.0, fontSize: 36, bold: true, color: C.background1, valign: 'middle', margin: 0 }, text: 'Title' } },
  ],
});

// ---------- helpers ----------
const txt = (slide, text, o) => slide.addText(text, Object.assign({ isTextBox: true, margin: 0, fontSize: 16, color: C.text1, valign: 'top' }, o));
const card = (slide, x, y, w, h, name) => slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
  x, y, w, h, rectRadius: 0.08, fill: { color: C.background2 }, line: { color: C.background2 }, objectName: name });
const numDot = (slide, x, y, n, color, name) => {
  slide.addShape(pres.shapes.OVAL, { x, y, w: 0.5, h: 0.5, fill: { color }, line: { color }, objectName: name });
  txt(slide, String(n), { x, y, w: 0.5, h: 0.5, fontSize: 16, bold: true, color: C.background1, align: 'center', valign: 'middle' });
};
const stat = (slide, x, y, w, big, small, color, name) => {
  txt(slide, big, { x, y, w, h: 0.95, fontSize: 48, bold: true, color: color || C.accent1, valign: 'bottom', objectName: name });
  txt(slide, small, { x, y: y + 1.0, w, h: 0.7, fontSize: 14, color: C.text2 });
};
const arrow = (slide, x, y, w, h, name) => slide.addShape(pres.shapes.LINE, {
  x, y, w, h, line: { color: HEX.accent5, width: 1.5, endArrowType: 'triangle' }, objectName: name });

// ================= Intro =================
pres.addSection({ title: 'Introduction' });

// 1. Title
{
  const s = pres.addSlide({ masterName: 'TITLE_DARK', sectionTitle: 'Introduction' });
  s.addText('Strata Cobot', { placeholder: 'title' });
  s.addText('Autonomous ore sorting and haulage with two robots in ROS 2', { placeholder: 'body' });
  txt(s, 'C Saihardhik Reddy  ·  R Gagan Chowdary  ·  K Charan Saatvikh Reddy', { x: 0.7, y: 5.75, w: 11.9, h: 0.4, fontSize: 16, bold: true, color: C.background1 });
  txt(s, 'Robotics course project  ·  Amrita Vishwa Vidyapeetham',
    { x: 0.7, y: 6.25, w: 11.9, h: 0.4, fontSize: 14, color: C.accent5 });
  s.addImage({ path: IMG('detection.png'), x: 7.6, y: 1.35, w: 5.1, h: 3.825, objectName: 'hero-camera-frame',
    altText: 'RealSense camera frame with four ores detected on the conveyor' });
  txt(s, 'The arm camera\'s view: four ores detected on the conveyor', { x: 7.6, y: 5.3, w: 5.1, h: 0.4, fontSize: 12, color: C.accent5 });
  s.addNotes('Our project: two robots, a UR7e arm and a mobile eBot, run a mining operation in simulation with no human in the loop. Everything you will see - perception, inverse kinematics, path planning - we wrote ourselves, without MoveIt2, Nav2 or any other ready-made planner.');
}

// 2. Mission
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Introduction' });
  s.addText('Two robots, one mining mission', { placeholder: 'title' });
  const steps = [
    ['eBot crosses the site', 'Plans its own route around the rocks to the excavation region'],
    ['eBot takes on the ore', 'Stops under the chute and collects a container of raw ore'],
    ['Arm sees each ore', 'An RGB-D camera finds every ore and its 3D position'],
    ['Arm sorts it', 'A magnet picks each ore and drops it in the container for its kind'],
  ];
  steps.forEach(([h, d], i) => {
    const y = 1.6 + i * 1.2;
    numDot(s, 0.6, y, i + 1, i < 2 ? C.accent6 : C.accent1, `step-${i + 1}-dot`);
    txt(s, h, { x: 1.3, y: y - 0.05, w: 6.0, h: 0.45, fontSize: 20, bold: true });
    txt(s, d, { x: 1.3, y: y + 0.42, w: 6.0, h: 0.5, fontSize: 15, color: C.text2 });
  });
  card(s, 7.9, 1.55, 4.8, 4.6, 'containers-card');
  txt(s, 'Three minerals, three containers', { x: 8.2, y: 1.8, w: 4.2, h: 0.45, fontSize: 18, bold: true });
  [['Azurite', 'blue face', C.accent2], ['Malachite', 'green face', C.accent3], ['Vanadinite', 'orange face', C.accent4]].forEach(([n, d, col], i) => {
    const y = 2.6 + i * 1.1;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 8.2, y, w: 0.75, h: 0.75, rectRadius: 0.06, fill: { color: col }, line: { color: col }, objectName: `ore-${n.toLowerCase()}` });
    txt(s, n, { x: 9.2, y: y + 0.02, w: 3.2, h: 0.4, fontSize: 18, bold: true });
    txt(s, d, { x: 9.2, y: y + 0.4, w: 3.2, h: 0.35, fontSize: 14, color: C.text2 });
  });
  s.addNotes('The story: the eBot owns mobility, the arm owns manipulation. We built and tested each half separately; handing ore from one robot to the other is future work. Each ore is a black block with one coloured top face, and that face is how both the camera and the scoring tell them apart.');
}

// 3. Project goals
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Introduction' });
  s.addText('Four goals, each with a number to check', { placeholder: 'title' });
  const head = ['Subsystem', 'Robot', 'Goal', 'Measured by'].map((t) => ({ text: t, options: { bold: true, color: HEX.lt1, fill: { color: HEX.dk1 } } }));
  const rows = [
    ['Perception', 'Arm camera', 'Locate every ore in 3D from colour and depth images', 'error vs. ground truth'],
    ['Manipulation', 'UR7e arm', 'Pick each ore with the magnet and drop it in its own container', 'ores sorted correctly'],
    ['Path planning', 'eBot', 'Plan a collision-free path for each leg of the lap', 'length vs. shortest'],
    ['Path following', 'eBot', 'Track each path and stop on every pose', 'tracking error'],
  ].map((r) => r.map((c) => ({ text: c })));
  s.addTable([head, ...rows], {
    x: 0.6, y: 1.55, w: 12.1, colW: [2.1, 1.9, 5.6, 2.5], fontSize: 16, fontFace: 'Calibri', color: HEX.dk1,
    border: { type: 'solid', pt: 0.75, color: 'D9D4CF' }, rowH: 0.7, valign: 'middle', margin: [0.06, 0.12, 0.06, 0.12],
  });
  txt(s, 'Constraint we set ourselves: no MoveIt2, no Nav2, no ready-made planner of any kind.',
    { x: 0.6, y: 5.3, w: 12.1, h: 0.6, fontSize: 16, bold: true, color: C.accent1 });
  s.addNotes('We split the project into four subsystems, two per robot, and gave each one a number we could check against the simulator rather than judging by eye.');
}

// 4. Architecture
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Introduction' });
  s.addText('Five nodes, two sense-plan-act pipelines and a hand-off', { placeholder: 'title' });
  const box = (x, y, w, h, title, lines, name, accent) => {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08, fill: { color: accent ? 'FBEDE6' : HEX.lt1 }, line: { color: accent ? HEX.accent1 : HEX.accent5, width: accent ? 1.75 : 1 }, objectName: name });
    txt(s, title, { x: x + 0.2, y: y + 0.12, w: w - 0.4, h: 0.35, fontSize: 16, bold: true });
    txt(s, lines.map((l, i) => ({ text: l, options: { breakLine: i < lines.length - 1 } })), { x: x + 0.2, y: y + 0.5, w: w - 0.4, h: h - 0.55, fontSize: 12.5, color: C.text2 });
  };
  box(0.6, 1.5, 2.6, 4.9, 'Gazebo Harmonic', ['UR7e arm + magnet', 'RealSense D435i', 'eBot + lidar', 'rocks and /map', '', 'ROS 2 Jazzy bridge'], 'node-sim');
  txt(s, 'Manipulator', { x: 4.4, y: 1.4, w: 3, h: 0.3, fontSize: 13, color: C.accent5 });
  box(4.4, 1.75, 3.4, 1.45, 'ore_perception', ['find each ore by depth and shape', 'name it by a colour vote', 'tf frame per ore'], 'node-perception');
  box(9.1, 1.75, 3.6, 1.45, 'arm_manipulation', ['damped least-squares IK', 'collision-checked routes', 'force-checked picks'], 'node-manipulation', true);
  box(9.1, 3.75, 3.6, 0.95, 'package_handoff', ['lifts the package off the eBot onto the conveyor'], 'node-handoff');
  txt(s, 'Mobile robot', { x: 4.4, y: 4.6, w: 3, h: 0.3, fontSize: 13, color: C.accent5 });
  box(4.4, 4.95, 3.4, 1.45, 'path_planning', ['clearance map', 'A*, shortcut, tighten', 'lap sequencing, re-planning'], 'node-planning', true);
  box(9.1, 4.95, 3.6, 1.45, 'path_following', ['pure pursuit', 'turn to final heading', 'stops on unmapped obstacles'], 'node-navigation');
  arrow(s, 3.2, 2.25, 1.2, 0, 'a-rgbd'); txt(s, 'RGB-D images', { x: 3.25, y: 1.9, w: 1.15, h: 0.3, fontSize: 11, color: C.text2, align: 'center' });
  arrow(s, 7.8, 2.25, 1.3, 0, 'a-tf'); txt(s, '/tf ore frames', { x: 7.8, y: 1.9, w: 1.3, h: 0.3, fontSize: 11, color: C.text2, align: 'center' });
  arrow(s, 3.2, 5.4, 1.2, 0, 'a-map'); txt(s, '/map, /odom', { x: 3.25, y: 5.05, w: 1.15, h: 0.3, fontSize: 11, color: C.text2, align: 'center' });
  arrow(s, 7.8, 5.4, 1.3, 0, 'a-path'); txt(s, '/ebot_path', { x: 7.8, y: 5.05, w: 1.3, h: 0.3, fontSize: 11, color: C.text2, align: 'center' });
  // lidar obstacles reported back to the planner
  s.addShape(pres.shapes.LINE, { x: 7.8, y: 5.95, w: 1.3, h: 0, line: { color: HEX.accent5, width: 1.5, beginArrowType: 'triangle' }, objectName: 'a-obstacles' });
  txt(s, '/ebot_obstacles', { x: 7.8, y: 6.0, w: 1.3, h: 0.3, fontSize: 11, color: C.text2, align: 'center' });
  // the hand-off between the eBot's planner and the arm
  s.addShape(pres.shapes.LINE, { x: 6.1, y: 4.25, w: 0, h: 0.7, line: { color: HEX.accent1, width: 1.5, endArrowType: 'triangle' }, objectName: 'h-1' });
  s.addShape(pres.shapes.LINE, { x: 6.1, y: 4.25, w: 3.0, h: 0, line: { color: HEX.accent1, width: 1.5, endArrowType: 'triangle' }, objectName: 'h-2' });
  txt(s, '/handoff/ready  ⇄  /handoff/done', { x: 6.2, y: 3.92, w: 2.9, h: 0.3, fontSize: 11, color: C.accent1 });
  // returns to the simulation
  s.addShape(pres.shapes.LINE, { x: 10.9, y: 3.2, w: 0, h: 0.3, line: { color: HEX.accent5, width: 1.5 }, objectName: 'r-arm-1' });
  s.addShape(pres.shapes.LINE, { x: 3.2, y: 3.5, w: 7.7, h: 0, line: { color: HEX.accent5, width: 1.5, beginArrowType: 'triangle' }, objectName: 'r-arm-2' });
  txt(s, '/delta_joint_cmds, /magnet', { x: 3.4, y: 3.55, w: 2.6, h: 0.3, fontSize: 11, color: C.text2 });
  s.addShape(pres.shapes.LINE, { x: 10.9, y: 6.4, w: 0, h: 0.3, line: { color: HEX.accent5, width: 1.5 }, objectName: 'r-ebot-1' });
  s.addShape(pres.shapes.LINE, { x: 3.2, y: 6.7, w: 7.7, h: 0, line: { color: HEX.accent5, width: 1.5, beginArrowType: 'triangle' }, objectName: 'r-ebot-2' });
  txt(s, '/cmd_vel  ·  /spawn_ore_package', { x: 4.4, y: 6.72, w: 4.5, h: 0.3, fontSize: 11, color: C.text2 });
  s.addNotes('Two Python nodes per robot, plus package_handoff, which extends the arm node for the full mission: the planner says the eBot is in place on /handoff/ready, the arm lifts the package onto the conveyor and answers on /handoff/done. The follower reports anything its lidar sees that the map lacks on /ebot_obstacles, and the planner re-plans. The arm accepts only velocity commands and the eBot only velocity twists, so every node closes its own control loop at 20 to 50 Hz. The two highlighted nodes hold most of the algorithmic work.');
}

// ================= Manipulator =================
pres.addSection({ title: 'The manipulator' });

// 5. Perception
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The manipulator' });
  s.addText('Find each ore by shape, name it by a colour vote', { placeholder: 'title' });
  s.addImage({ path: IMG('detection.png'), x: 0.6, y: 1.5, w: 6.4, h: 4.8, objectName: 'camera-detections',
    altText: 'Camera frame with azurite, malachite and two vanadinite ores outlined on the conveyor' });
  stat(s, 7.5, 1.5, 2.5, '2-3 mm', 'position error vs. ground truth', C.accent1, 'stat-error');
  stat(s, 10.2, 1.5, 2.5, '10 mm', 'about what the magnet tolerates', C.accent5, 'stat-tolerance');
  const pts = [
    'Where: an ore\'s top face is the only surface 66-86 mm above the belt in the depth image',
    'Centre of a rectangle fitted to that face, checked against the 101.6 mm size',
    'What: the share of the face in each mineral\'s colour; the winner needs 2x the next',
    'Majority vote over 20 frames; no clear winner = "uncertain", never a guess',
  ];
  txt(s, pts.map((p, i) => ({ text: p, options: { bullet: true, breakLine: i < pts.length - 1 } })),
    { x: 7.5, y: 3.5, w: 5.2, h: 2.9, fontSize: 15, paraSpaceAfter: 8 });
  s.addNotes('Our first detector found ores by tight colour thresholds. Now we separate where an ore is from what it is. Where comes from the depth camera: every ore is a block standing 76 mm above the belt, so its top face is found whatever its colour. What comes from a vote: we count how much of the face shows each mineral\'s colour. Fitting a rectangle to the face, instead of taking a median, keeps the position within 2 to 3 mm.');
}

// 5b. Impure ores
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The manipulator' });
  s.addText('Real ores are rarely one colour, so we vote', { placeholder: 'title' });
  s.addImage({ path: IMG('impurity_robustness.png'), x: 0.5, y: 1.45, w: 7.2, h: 4.11, objectName: 'impurity-chart',
    altText: 'Accuracy against impurity: the voting detector stays at 100% up to 50% impurity while tight thresholds fall to 48% at 20%' });
  txt(s, 'Impurities painted into a real camera frame: dark rock, brown matrix, quartz and veins of other minerals, under varied lighting. 160 ores per level.',
    { x: 0.5, y: 5.7, w: 7.2, h: 0.7, fontSize: 13, color: C.text2 });
  txt(s, '180 real mineral photos', { x: 8.1, y: 1.5, w: 4.6, h: 0.4, fontSize: 16, bold: true });
  const head = ['Held-out photos', 'Tight', 'Vote'].map((t) => ({ text: t, options: { bold: true, color: HEX.lt1, fill: { color: HEX.dk1 } } }));
  const rows = [['Azurite', '10%', '37%'], ['Malachite', '7%', '80%'], ['Vanadinite', '17%', '93%'], ['All', '11%', '70%']]
    .map((r, i) => r.map((c, j) => ({ text: c, options: { bold: i === 3, color: j === 2 ? HEX.accent1 : HEX.dk1 } })));
  s.addTable([head, ...rows], { x: 8.1, y: 2.0, w: 4.6, colW: [2.2, 1.2, 1.2], fontSize: 15, fontFace: 'Calibri',
    border: { type: 'solid', pt: 0.75, color: 'D9D4CF' }, rowH: 0.5, valign: 'middle', margin: [0.05, 0.12, 0.05, 0.12] });
  txt(s, 'Museum photos from Wikimedia Commons. Limits calibrated on 90, tested on 90 others. Azurite is hardest: very dark, and often crusted with green malachite.',
    { x: 8.1, y: 4.75, w: 4.6, h: 1.2, fontSize: 13, color: C.text2 });
  s.addNotes('Our lecturer pointed out that real ores are not pure colours. We tested that two ways. First, we painted impurities into a real camera frame: the old tight thresholds fall below half at 20 percent impurity, while the vote stays perfect up to 50 percent and never picks the wrong mineral. Second, we ran both on 180 real mineral photographs: tight thresholds recognise 11 percent, the vote 70 percent on photos it was not tuned on. Azurite is hardest, because real azurite is very dark and usually mixed with malachite. A trained classifier is the next step once real samples are available.');
}

// 6. Containers
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The manipulator' });
  s.addText('The containers were found by measuring the rover mesh', { placeholder: 'title' });
  s.addImage({ path: IMG('workspace.png'), x: 0.6, y: 1.4, w: 5.5, h: 5.13, objectName: 'workspace-top-view',
    altText: 'Top view of the arm workspace with conveyor, crusher, cargo rover containers and drop points' });
  const rows = [
    ['Not given to us', 'The cargo rover\'s collision mesh (GLB) was parsed and moved into base_link: three containers, 0.26-0.31 m by 0.40 m.'],
    ['Which is which', 'Gazebo\'s default camera looks along +x: vanadinite nearest, malachite middle, azurite farthest.'],
    ['One is out of reach', 'The vanadinite centre is 0.93 m from the base. Each ore is released at the nearest point inside its container, 8 cm from the walls.'],
  ];
  rows.forEach(([h, d], i) => {
    const y = 1.55 + i * 1.65;
    card(s, 6.7, y, 6.0, 1.45, `fact-${i + 1}`);
    txt(s, h, { x: 6.95, y: y + 0.15, w: 5.5, h: 0.4, fontSize: 18, bold: true, color: i === 2 ? C.accent1 : C.text1 });
    txt(s, d, { x: 6.95, y: y + 0.58, w: 5.5, h: 0.8, fontSize: 14, color: C.text2 });
  });
  s.addNotes('The container positions were not given. We parsed the rover\'s collision mesh to get exact walls. The surprise: the vanadinite container\'s centre is beyond the arm\'s reach with the tool vertical, so every ore is dropped at the reachable edge of its container. The crosses on the plot are the drop points.');
}

// 7. Pick sequence
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The manipulator' });
  s.addText('Every pick is confirmed by the force sensor', { placeholder: 'title' });
  const steps = [
    ['Hover', '10 cm above the face, tool vertical'],
    ['Align', 'close the last mm on /tcp_pose_raw; magnet on'],
    ['Descend', '4 then 1.2 cm/s; stop at the face or 2.5 N'],
    ['Lift + check', '1.47 N = ore held; under 0.8 N, retry'],
    ['Drop', 'carry with the ore in the collision model; release'],
  ];
  const w = 2.2, gap = 0.275;
  steps.forEach(([h, d], i) => {
    const x = 0.6 + i * (w + gap);
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.9, w, h: 2.3, rectRadius: 0.08, fill: { color: i === 3 ? 'FBEDE6' : HEX.lt2 }, line: { color: i === 3 ? HEX.accent1 : HEX.lt2, width: 1.5 }, objectName: `pick-step-${i + 1}` });
    numDot(s, x + 0.2, 2.1, i + 1, C.accent1, `pick-dot-${i + 1}`);
    txt(s, h, { x: x + 0.2, y: 2.75, w: w - 0.4, h: 0.45, fontSize: 18, bold: true });
    txt(s, d, { x: x + 0.2, y: 3.25, w: w - 0.4, h: 0.9, fontSize: 14, color: C.text2 });
    if (i < steps.length - 1) arrow(s, x + w + 0.02, 3.05, gap - 0.04, 0, `pick-arrow-${i + 1}`);
  });
  stat(s, 0.6, 4.75, 3.6, '81 mm', 'magnet disc on a 101.6 mm face: about 10 mm to spare', C.accent1, 'stat-magnet');
  stat(s, 4.7, 4.75, 3.6, '1.47 N', 'force reading for a held 0.15 kg ore', C.accent1, 'stat-force');
  stat(s, 8.8, 4.75, 3.9, '0 stops', 'safety stops across every trial', C.accent3, 'stat-stops');
  s.addNotes('The magnet only captures when its face is fully inside the ore face and parallel to it, so the descent is a straight vertical line with the tool held vertical. We do not trust the magnet service reply: after lifting, the force sensor must read the ore\'s weight, about 1.47 N, or the pick is retried. If a protective stop ever latches, the node unlocks, backs off upward and re-plans - but none happened in any trial.');
}

// 8. IK
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The manipulator' });
  s.addText('A purpose-built IK solver plans 300x faster', { placeholder: 'title' });
  card(s, 0.6, 1.6, 3.6, 2.6, 'before-card');
  txt(s, 'Before: SLSQP optimiser', { x: 0.85, y: 1.8, w: 3.2, h: 0.4, fontSize: 16, bold: true, color: C.text2 });
  txt(s, 'minutes', { x: 0.85, y: 2.3, w: 3.2, h: 1.0, fontSize: 48, bold: true, color: C.accent5 });
  txt(s, 'per plan; stuck when starting above the containers', { x: 0.85, y: 3.35, w: 3.2, h: 0.7, fontSize: 14, color: C.text2 });
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 4.6, y: 1.6, w: 3.6, h: 2.6, rectRadius: 0.08, fill: { color: 'FBEDE6' }, line: { color: HEX.accent1, width: 1.75 }, objectName: 'after-card' });
  txt(s, 'After: damped least squares', { x: 4.85, y: 1.8, w: 3.2, h: 0.4, fontSize: 16, bold: true, color: C.accent1 });
  txt(s, '0.1-0.4 s', { x: 4.85, y: 2.3, w: 3.2, h: 1.0, fontSize: 48, bold: true, color: C.accent1 });
  txt(s, 'per plan, with every safety check kept', { x: 4.85, y: 3.35, w: 3.2, h: 0.7, fontSize: 14, color: C.text2 });
  const pts = [
    ['5 constraints, not 6', 'Tool position + vertical axis; rotation about vertical is free because the magnet is round'],
    ['Checks once, not every step', 'Limits, 3 cm clearance, singularity and tilt are tested on the converged result'],
    ['A memory of solutions', 'Every solved pose is reused as a seed, so the arm always finds its way back'],
    ['Straight vertical moves', 'Resolved-rate control: tool velocity mapped to joints through the Jacobian'],
  ];
  pts.forEach(([h, d], i) => {
    const y = 1.6 + i * 1.22;
    txt(s, h, { x: 8.7, y, w: 4.0, h: 0.4, fontSize: 17, bold: true });
    txt(s, d, { x: 8.7, y: y + 0.4, w: 4.0, h: 0.75, fontSize: 13.5, color: C.text2 });
  });
  const sup = (t) => ({ text: t, options: { superscript: true } });
  txt(s, [{ text: 'Δq = J' }, sup('T'), { text: ' (J J' }, sup('T'), { text: ' + λ' }, sup('2'), { text: ' I)' }, sup('−1'), { text: ' e,   λ = 0.02' }],
    { x: 0.6, y: 4.75, w: 7.6, h: 0.6, fontSize: 22, color: C.text1, objectName: 'dls-equation' });
  txt(s, 'e = position error and tool-axis tilt; J = the matching 5 rows of the arm\'s Jacobian', { x: 0.6, y: 5.4, w: 7.6, h: 0.5, fontSize: 14, color: C.text2 });
  s.addNotes('Our first IK used a general constrained optimiser, and every gradient step re-ran the collision and singularity checks: minutes per plan. Damped least squares solves only what the problem needs - five constraints - and checks safety once at the end. Add a library of solved poses as seeds and planning drops to a fraction of a second.');
}

// ================= Mobile robot =================
pres.addSection({ title: 'The mobile robot' });

// 9. Planner
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The mobile robot' });
  s.addText('A*, then shortcut, then tighten', { placeholder: 'title' });
  s.addImage({ path: IMG('paths.png'), x: 0.5, y: 1.35, w: 7.6, h: 5.24, objectName: 'planned-paths',
    altText: 'Three planned paths on the arena occupancy grid between home, ore drop and arm poses' });
  const steps = [
    ['Clearance map', 'Distance transform of the 5 mm /map: every cell knows its distance to a rock'],
    ['A* search', '8-connected on a 5 cm grid, over cells that keep the chosen clearance'],
    ['Shortcut', 'Jump to the furthest point visible with full clearance'],
    ['Tighten', 'Pull each corner toward its neighbours until the legs just clear'],
  ];
  steps.forEach(([h, d], i) => {
    const y = 1.55 + i * 1.25;
    numDot(s, 8.5, y, i + 1, C.accent1, `plan-dot-${i + 1}`);
    txt(s, h, { x: 9.2, y: y - 0.02, w: 3.5, h: 0.4, fontSize: 18, bold: true });
    txt(s, d, { x: 9.2, y: y + 0.4, w: 3.5, h: 0.75, fontSize: 13.5, color: C.text2 });
  });
  s.addNotes('Home to the ore drop pose, then to the arm pose, then home. Every straight leg is re-checked on the full-resolution map. Tightening matters: without it, corners sit on 5 cm grid cells and the path is a few percent too long - which made our first version 2 percent too long.');
}

// 10. Clearance trade-off (native chart)
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'The mobile robot' });
  s.addText('Below 0.40 m clearance, paths stop getting shorter', { placeholder: 'title' });
  s.addChart(pres.charts.LINE, [{ name: 'Lap length (m)',
    labels: ['0.34', '0.36', '0.38', '0.395', '0.42', '0.45'], values: [14.45, 14.46, 14.47, 14.49, 15.14, 16.34] }], {
    x: 0.6, y: 1.45, w: 7.4, h: 4.9, chartColors: [HEX.accent1], lineSize: 2.5, lineDataSymbol: 'circle', lineDataSymbolSize: 8,
    showValue: true, dataLabelPosition: 't', dataLabelFormatCode: '0.00', dataLabelFontSize: 12, dataLabelColor: HEX.dk1, dataLabelFontFace: '+mn-lt',
    showTitle: true, title: 'Total lap length vs. clearance kept from rocks', titleFontSize: 14, titleColor: HEX.dk1, titleFontFace: '+mn-lt',
    catAxisTitle: 'Clearance from body centre to rock (m)', showCatAxisTitle: true, catAxisTitleFontSize: 12, catAxisTitleColor: HEX.accent5,
    catAxisLabelColor: HEX.accent5, valAxisLabelColor: HEX.accent5, catAxisLabelFontFace: '+mn-lt', valAxisLabelFontFace: '+mn-lt',
    valAxisMinVal: 14, valAxisMaxVal: 16.5, valAxisLabelFormatCode: '0.0', valGridLine: { color: 'E5E1DD', size: 0.5 }, catGridLine: { style: 'none' },
    showLegend: false, objectName: 'tradeoff-chart',
  });
  stat(s, 8.5, 1.5, 4.2, '0.395 m', 'reference: a circle enclosing the 0.62 x 0.49 m chassis', C.accent5, 'stat-circle');
  stat(s, 8.5, 3.15, 4.2, '0.245 m', 'what really has to clear driving straight: the half-width', C.accent1, 'stat-halfwidth');
  txt(s, 'The planner estimates the reference path at 0.395 m, then takes the safest level from 0.40 to 0.34 m that is no longer. At 0.38 m the eBot keeps 13.5 cm each side; it tracked within 9 cm.',
    { x: 8.5, y: 4.9, w: 4.2, h: 1.5, fontSize: 14, color: C.text2 });
  s.addNotes('We measure path quality against the shortest path for a 0.395 m circle that encloses the chassis. But when driving straight past a rock only the half-width must clear, so we can plan slightly inside that clearance and match the shortest length while keeping 13 cm to spare. Above 0.40 m narrow gaps close and the lap grows by up to 13 percent.');
}

// ================= Results =================
pres.addSection({ title: 'Results' });

// 11. Results
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Results' });
  s.addText('Every ore sorted, every lap completed, no collisions', { placeholder: 'title' });
  stat(s, 0.6, 1.5, 3.5, '8 / 8', 'ores sorted into the right container, two trials', C.accent3, 'stat-sorted');
  stat(s, 0.6, 3.35, 3.5, '1.00', 'best path efficiency (reference / our length)', C.accent3, 'stat-efficiency');
  txt(s, 'No collision and no safety stop in any trial', { x: 0.6, y: 5.25, w: 3.5, h: 0.8, fontSize: 15, bold: true, color: C.text1 });
  const head = ['Trial', 'Outcome'].map((t) => ({ text: t, options: { bold: true, color: HEX.lt1, fill: { color: HEX.dk1 } } }));
  const rows = [
    ['Arm 1', '4/4 ores detected, picked and dropped in the right container'],
    ['Arm 2', 'New ore mix, one ore 0.83 m away: 4/4 sorted in 167 s'],
    ['Robot 1', 'First planner: 14.74 m lap vs 14.46 m reference (efficiency 0.98)'],
    ['Robot 2', 'Final planner: 14.55 m vs 14.58 m (efficiency 1.00)'],
    ['Robot 3', 'Final planner: 14.48 m vs 14.43 m (efficiency 0.996)'],
  ].map((r) => r.map((c) => ({ text: c })));
  s.addTable([head, ...rows], { x: 4.6, y: 1.55, w: 8.1, colW: [1.3, 6.8], fontSize: 15, fontFace: 'Calibri', color: HEX.dk1,
    border: { type: 'solid', pt: 0.75, color: 'D9D4CF' }, rowH: 0.6, valign: 'middle', margin: [0.05, 0.12, 0.05, 0.12] });
  txt(s, 'Robot 3 falls short only because the reference itself moved: on the same rocks it measured 6.14 m in one trial and 6.07 m in the next.',
    { x: 4.6, y: 5.4, w: 8.1, h: 0.8, fontSize: 14, color: C.text2 });
  s.addNotes('Every trial used a freshly launched world with randomised ores or rocks, checked by the automatic evaluator that comes with the simulation. Sorting was perfect in both trials on different ore mixes. Path efficiency reached 1.00; the small shortfall in one trial comes from the reference path varying between runs.');
}

// 11b. Hand-off and re-planning
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Results' });
  s.addText('The full loop: hand-off and re-planning', { placeholder: 'title' });
  s.addImage({ path: IMG('dashboard_mission.jpg'), x: 0.6, y: 1.45, w: 5.9, h: 3.69, objectName: 'mission-shot',
    altText: 'Dashboard after a full mission: all five stages ticked, the package on the conveyor' });
  s.addImage({ path: IMG('dashboard_replan.jpg'), x: 6.8, y: 1.45, w: 5.9, h: 4.43, objectName: 'replan-shot',
    altText: 'Dashboard map with an abandoned path, a surprise rock and the new path around it' });
  txt(s, 'Full mission, one click, about 2 min: the eBot brings the ore package, the arm lifts it onto the conveyor (2.94 N = 0.30 kg held), the eBot drives home.',
    { x: 0.6, y: 5.3, w: 5.9, h: 1.2, fontSize: 14, color: C.text2 });
  txt(s, 'Surprise rock: the lidar sees it within 0.26 m of the path, the eBot stops and the planner re-plans in 0.1 s.',
    { x: 6.8, y: 6.0, w: 5.9, h: 0.9, fontSize: 14, color: C.text2 });
  s.addNotes('Two things the first version could not do. First, the full loop: the eBot hauls the ore package to the arm, the arm finds it with odometry and the depth camera, lifts it off the deck and places it on the conveyor, then the eBot drives home. Second, obstacles the map does not have: our paths always keep 30 cm from mapped rocks, so anything the lidar sees closer than 26 cm to the path is new. The robot stops, the planner adds it to the map and re-plans in a tenth of a second.');
}

// 12. Challenges grid
{
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: 'Results' });
  s.addText('What broke, and how we fixed it', { placeholder: 'title' });
  const items = [
    ['No path at all', 'Drop pose is 0.375 m from a wall; downsampling shaved 5 cm more. Full-resolution clearance fixed it.'],
    ['Paths 2% too long', 'Grid-snapped corners. Fine-map checks and corner tightening reached efficiency 1.00.'],
    ['IK took minutes', 'Checks inside every optimiser step. DLS with one final check: 0.1-0.4 s.'],
    ['Arm lost after a drop', 'No IK seed back to the conveyor. A library of solved poses fixed it.'],
    ['Vanadinite unreachable', 'Container centre 0.93 m away. Release at its nearest reachable point.'],
    ['Two simulations clashed', 'A leftover simulation on the same ROS domain. One world at a time.'],
  ];
  items.forEach(([h, d], i) => {
    const col = i % 3, row = Math.floor(i / 3);
    const x = 0.6 + col * 4.1, y = 1.55 + row * 2.6;
    card(s, x, y, 3.85, 2.35, `issue-${i + 1}`);
    txt(s, h, { x: x + 0.25, y: y + 0.22, w: 3.35, h: 0.45, fontSize: 18, bold: true, color: C.accent1 });
    txt(s, d, { x: x + 0.25, y: y + 0.75, w: 3.35, h: 1.45, fontSize: 14.5, color: C.text1 });
  });
  s.addNotes('Most of these were invisible until we ran full trials with automatic checking, which is the main lesson: test end to end early. The others came from measuring the geometry ourselves instead of trusting the brief.');
}

// 13. Closing
{
  const s = pres.addSlide({ masterName: 'CLOSING_DARK', sectionTitle: 'Results' });
  s.addText('Both robots complete their missions end to end', { placeholder: 'title' });
  const done = ['Sorting: 16 of 16 ores in the right container', 'Planning: paths as short as the reference', 'Hand-off: eBot to arm to conveyor', 'Re-planning around unmapped rocks', 'Every algorithm written from scratch'];
  done.forEach((d, i) => {
    numDot(s, 0.7, 1.9 + i * 0.82, i + 1, C.accent1, `done-${i + 1}`);
    txt(s, d, { x: 1.45, y: 1.93 + i * 0.82, w: 5.6, h: 0.7, fontSize: 18, color: C.background1, valign: 'top' });
  });
  txt(s, 'Next', { x: 7.4, y: 1.95, w: 5.2, h: 0.5, fontSize: 22, bold: true, color: C.accent4 });
  const next = ['Train a classifier on real ore samples', 'Move to physical robots'];
  txt(s, next.map((n, i) => ({ text: n, options: { bullet: true, breakLine: i < next.length - 1 } })),
    { x: 7.4, y: 2.6, w: 5.2, h: 2.8, fontSize: 18, color: C.background2, paraSpaceAfter: 10 });
  txt(s, 'Thank you  ·  Questions?', { x: 0.7, y: 6.2, w: 11.9, h: 0.6, fontSize: 20, bold: true, color: C.background1 });
  s.addNotes('To sum up: both halves of the mission work end to end in simulation, using only algorithms we wrote. The two robots also work together: the eBot delivers the ore package to the arm, which lifts it onto the conveyor, and the eBot re-plans around rocks it did not expect. The natural next steps are a classifier trained on real ore samples and physical robots. Happy to take questions, and we can run a live demo.');
}

pres.writeFile({ fileName: OUT }).then(async () => {
  const { applyTheme } = require(process.env.PPTX_SKILL + '/scripts/apply_theme.js');
  await applyTheme(OUT, THEME);
  console.log('wrote', OUT);
});
