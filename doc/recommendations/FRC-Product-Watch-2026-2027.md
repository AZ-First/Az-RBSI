# FRC Product Watch: REBUILT 2026 and BIOCORE 2027

**Research snapshot:** August 29, 2026

**Scope:** Chief Delphi product threads, 2026 Open Alliance reports, vendor
postmortems, and FIRST control-system announcements.

## Recommendation In Brief

Az-RBSI teams should treat the products discussed below in three different
ways:

1. **Standardize selectively:** integrated brushless motors and mature vision
   hardware are proven enough to standardize where they match an existing
   vendor ecosystem.
2. **Buy only with a design reason:** low-profile swerve modules and specialty
   roller materials were important in REBUILT, but much of their advantage was
   specific to the game's packaging and FUEL-handling demands.
3. **Prototype before committing:** SystemCore is the central 2027 platform
   transition. New vision hardware, motor-controller alternatives, and beta
   libraries should be evaluated on a test robot before the team buys a fleet
   or makes them part of Az-RBSI's default configuration.

The strongest practical recommendation is to spend the 2026 offseason making
Az-RBSI and one representative robot fully SystemCore-ready. Do not let
interesting announcements consume the budget needed for that transition.

## How “Hot” Was Determined

Chief Delphi attention is not the same as market share or reliability. This
review uses three signals:

- unusually large discussion threads or strong launch reaction;
- repeated appearance in unrelated 2026 build reports; and
- post-season reports describing actual competition use, including failures.

Vendor mega-threads are evidence of interest in an ecosystem, not proof that
every item in the thread was popular. For example, the 2025–2026 REV and
Thrifty Bot release threads each accumulated hundreds of replies, while the
[SDS MK5n thread](https://www.chiefdelphi.com/t/sds-mk5n-swerve-module/501256)
alone grew past 600 replies. Recommendations therefore emphasize independent
team use and postmortems over launch enthusiasm.

## Products That Stood Out During REBUILT

### 1. SDS MK5n and MK5i Low-Profile Swerve Modules

**Attention level: very high. Adoption evidence: strong. Recommendation:
consider for a new drivetrain, not as an automatic upgrade.**

The MK5 family was the clearest individual hardware attention story. The MK5n
launch drew a large and sustained technical discussion, and the
[MK5i announcement](https://www.chiefdelphi.com/t/sds-mk5i-swerve-module/504736)
added a version with a familiar corner footprint. Multiple Open Alliance teams
selected or competed with MK5 modules during REBUILT.

The attraction was not simply novelty. A lower and narrower module opened more
volume for low hopper floors and wide intakes, while the enclosed geartrain and
changeable ratios improved serviceability. Those packaging benefits happened
to align unusually well with a game requiring teams to collect and store many
pieces.

There were also real first-season lessons. Teams reported damaged molded wheel
hubs after impacts, documented in the
[MK5n competition discussion](https://www.chiefdelphi.com/t/sds-mk5n-swerve-module/501256/579),
while another user reported running from week one with little maintenance.
That is a reason to stock wheels and inspect hubs, not evidence that the module
is broadly unreliable.

**Implementation recommendation:**

- Keep existing MK4-class modules if they are reliable and already supported.
- Choose MK5n when its smaller footprint changes the robot architecture; choose
  MK5i when the conventional corner layout simplifies the frame.
- Before competition, run curb/bump impacts and a full-event endurance test,
  then define a wheel/hub inspection interval and stock complete wear spares.
- Maintain both module geometries as configuration options in Az-RBSI rather
  than making either a universal default.

### 2. Kraken X60/X44 and the Integrated-Controller Motor Ecosystem

**Attention level: high. Adoption evidence: strong. Recommendation: a sound
standardization choice for teams already using Phoenix 6.**

Kraken motors appeared throughout competitive 2026 mechanisms. For example,
[Team 581's release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)
describes X60-powered intake, conveyor, and shooter systems plus X44-powered
deployment and hood adjustment. The compact X44 also enabled low-profile
steering arrangements in several new swerve products.

Teams repeatedly valued the integrated controller, common Phoenix tooling, and
reduced wiring. That is an ecosystem advantage, not proof that a Kraken is the
best motor for every load. Availability remained part of the conversation, and
WCP's first 2027 update said new X44 and X60 production was still rolling toward
stock in September 2026. See the
[WCP 2026–2027 release thread](https://www.chiefdelphi.com/t/wcp-2026-27-product-release-updates/523601).

**Implementation recommendation:**

- Standardize on one primary smart-motor ecosystem per robot to reduce spare,
  firmware, API, and pit-training burden.
- For an Az-RBSI team already committed to Phoenix 6, use X60 for high-power
  work and X44 where mass and envelope matter, after mechanism calculations.
- Order critical competition spares early and retain at least one known-good
  firmware/configuration image.
- Keep vendor-neutral subsystem interfaces in Az-RBSI so a team with legacy REV
  inventory can remain supported without mixing APIs inside mechanism logic.

### 3. Luma P1 and LumaSwitch

**Attention level: high. Adoption evidence: meaningful but mixed.
Recommendation: do not bulk-buy the first generation now; evaluate its
successor.**

Luma's pitch—PhotonVision appliance convenience at a lower multi-camera cost—
was compelling. Teams reported quick setup, stable operation, and the ability
to afford wider camera coverage. Other teams reported a dead-on-arrival unit,
network/reimaging trouble, fragile camera-cable insulation, or a vulnerable
USB connector.

Luma documented those failure modes openly in its
[2026 postmortem](https://www.chiefdelphi.com/t/luma-vision-2026-postmortem-and-feedback/519431).
The same thread says the P1 and LumaSwitch are being replaced by a new
generation, with active PoE under development and demand for a more capable
object-detection product.

**Implementation recommendation:**

- Freeze first-generation P1 purchases except for replacement of an installed
  system.
- Obtain one next-generation unit when final hardware and 2027 software exist.
- Bench-test cold boot, repeated power interruption, cable strain, network
  recovery, thermal behavior, time synchronization, and AprilTag latency.
- Compare it with an established Limelight 4 and a team-built PhotonVision
  coprocessor using the same camera placement and dataset.
- Preserve Az-RBSI's camera abstraction so the choice is a deployment decision,
  not a rewrite of localization or command code.

### 4. Cat Tongue and Silicone-Covered Tube Rollers

**Attention level: high for REBUILT mechanisms. Adoption evidence: strong but
game-specific. Recommendation: add to the prototyping library, not the default
bill of materials.**

Grip-covered tube rollers became a recurring REBUILT solution for lightweight,
full-width intakes, conveyors, and shooters. Teams 581, 157, 2102, 5826, and
others documented Cat Tongue or similar non-abrasive grip tape. The
[community test discussion](https://www.chiefdelphi.com/t/what-is-the-interaction-between-fuel-and-grip-tape/513734)
found very high grip, but also reported heat and deformation when a powered
roller jammed. Teams also reported damaged tape during defensive impacts.

**Implementation recommendation:** stock tape, silicone sleeving, polycarbonate
tube, and inexpensive roller hubs for kickoff prototypes. Test current draw,
jam heating, seam retention, wear, and game-piece damage before selecting a
surface. Add software current limits and jam reversal wherever a high-grip
roller can stall.

### 5. FRCDesignApp and FRCDesignLib

**Attention level: high among Onshape users. Adoption evidence: established
beta use. Recommendation: adopt as a design-team tool with normal CAD release
discipline.**

The new FRCDesignApp modernized the MKCad insertion workflow and its announcement
reported more than 15,000 part insertions by over 200 closed-beta users. The
[launch thread](https://www.chiefdelphi.com/t/introducing-the-new-frcdesignapp/507335)
also documented migration concerns such as product-number search and library
coverage.

This is low-risk to pilot because it does not become a runtime robot dependency.
Teams should still verify vendor drawings, pin configurations, mass, and revision
before releasing a design.

## BIOCORE 2027 Product Chatter and Watch List

### SystemCore: Plan as a Required Platform Transition

SystemCore generated far more consequential discussion than any ordinary
product. More than 400 eligible applications competed for 50 places in the
[first alpha wave](https://www.chiefdelphi.com/t/frc-blog-systemcore-alpha-testing-first-wave/503108).
FIRST later reported that teams had run it at offseason events, WPILib and
Limelight were addressing software feedback, hardware failures were being
root-caused, and the target price remained below the roboRIO. See the
[testing update](https://www.chiefdelphi.com/t/frc-blog-2027-control-system-testing-reminder/506964).

The community discussion contains speculation mixed with confirmed facts.
Treat FIRST/WPILib release notes and the 2027 manual as authoritative for final
legality and compatibility. Do not replace otherwise useful CAN devices merely
because the main controller changes.

**Az-RBSI action now:**

1. Maintain a SystemCore CI/build lane using the current 2027 WPILib alpha.
2. Run one complete representative robot or electronics board on SystemCore.
3. Exercise CAN devices, cameras, logging, USB, radio, Driver Station, deploy,
   code restart, brownout recovery, and field-like enable/disable cycles.
4. Write a pit recovery procedure and create a known-good image/configuration.
5. Reserve budget for competition and spare controllers before optional new
   mechanisms.

### A301 and MotionCore: Interesting, but Do Not Confuse FTC and FRC Plans

The A301 announcement produced extensive discussion because it combines a
compact brushless actuator, integrated output encoder, swappable gearboxes, and
CAN operation. FIRST plans for A301 to be FRC-legal in 2027, but MotionCore and
the exclusive-actuator transition described in the
[A301 announcement](https://www.chiefdelphi.com/t/ftc-frc-blog-introducing-the-first-a301/508970)
primarily concern FTC.

For Az-RBSI, A301 belongs on the evaluation list only after final FRC legality,
price, availability, API, performance curves, and mechanical ecosystem are
published. Buy one for characterization if it fills a real low-power actuator
gap; do not redesign FRC architecture around FTC-only MotionCore assumptions.

### Next-Generation Luma Hardware: High-Priority Evaluation

This is the most concrete 2027 vision watch item because the vendor explicitly
announced a replacement generation after receiving a season of feedback. Wait
for production specifications and independent test reports, then use the bench
procedure above. Active PoE could materially simplify camera wiring, but only if
the complete power path, connectors, recovery behavior, and rules compliance
are satisfactory.

### PlanetaryX, Pulsar 775/Nova, and Other Vendor Announcements: Watch, Then Test

WCP has teased a PlanetaryX family including PXS and Cyloid. Thrifty Bot released
the compact
[Pulsar 775](https://www.chiefdelphi.com/t/the-thrifty-bot-2025-2026-product-releases-updates/505971?page=3)
as a standalone brushless X44-class alternative and has published a 2027 beta
SystemCore API for Nova. These are credible reasons to watch, but the available
Chief Delphi material does not yet establish broad 2027 FRC adoption or
competition reliability.

Evaluate such products with the same gate:

- final price and in-season availability;
- published performance and thermal limits;
- SystemCore firmware/API maturity and simulation support;
- mechanical compatibility and repair time;
- independent endurance data; and
- whether the product reduces the team's total ecosystem count.

### YAGSL and Other 2027 Software Betas: Use a Test Branch

YAGSL's 2027 beta is adapting to SystemCore and a brushless-focused hardware
set. CTRE likewise announced that Phoenix 6 development had moved to SystemCore
while intending to keep the API close to 2026; its
[2026 roadmap](https://www.chiefdelphi.com/t/ctre-2026-phoenix-feedback-and-roadmap/521353)
also added mechanism-code generation, simulation, and WPILib logging
integration.

These changes are valuable but should live in an Az-RBSI 2027 integration branch
until compatible releases and hardware pass the team's regression suite. Pin
versions; do not build kickoff plans on an unpinned alpha.

## Proposed Purchasing Priorities

| Priority | Purchase or effort | Decision |
| --- | --- | --- |
| 1 | SystemCore development unit, competition unit, and spare | Budget first; validate the whole stack. |
| 2 | Spares for the team's existing motor/drive ecosystem | Preserve reliability before expanding ecosystems. |
| 3 | One next-generation Luma unit or equivalent vision candidate | Run a controlled comparison; no fleet purchase yet. |
| 4 | High-grip roller prototype materials | Low-cost, useful prototype inventory. |
| 5 | MK5 modules | Buy only when packaging or a new drivetrain justifies them. |
| 6 | A301, PlanetaryX, Pulsar/Nova, and other new releases | One-unit evaluation after final specs and APIs. |

## Acceptance Gate for Any New Product

A product should become an Az-RBSI recommendation only after it passes all of
the following:

- a named design problem that it solves better than existing inventory;
- at least one representative mechanism or robot test;
- field-like endurance and induced-fault testing;
- documented firmware, configuration, and rollback steps;
- a competition spare and replacement plan;
- support in logging, simulation, and automated tests where applicable; and
- student pit training that demonstrates replacement under time pressure.

This gate preserves the useful part of Chief Delphi product chatter—early
awareness and shared lessons—without treating popularity as engineering proof.
