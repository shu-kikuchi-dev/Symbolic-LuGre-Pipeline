%% --- LuGre Master Data Factory (version2.0, 2026-09-21) ---
% In this program, we will use only direct velocity input models as data
% generating factory. Previous one, we ve used a Spring-Mass system model
% in addition to the velocity model simulates pre-sliding, hysteresis.
% However, that Stick-Slip motion that Spring-Mass system provides is too
% tricky, chaotic to be treated as data to feed to AI. Because it has
% a long stacking time and break-way happens almost like impulse function.
% The extended logic has been explained at the last part of the note in
% 2026-09-03.

% With this program, we will try to observe mainly three phenomenon.
% 1. Pre-Sliding, Hysteresis: Not breaking-away moments. The bristle make
% complete loops. Observe this through high speed tiny sine waves.
% 2. Stribeck-Curve: Moments contain break-away, bristle growing and
% dropping with proper density, slow speed. With slowly growing ramp.
% 3. Viscous Friction: Observe the moment that the viscous friction has
% dominance. Through huge amplitude sine wave that moves slow.
% The detailed explanation about the each value calculation and entire 
% logic of selecting its ways based on a LuGre model theory has been 
% explained at the note written in 2026-09-18.

% Note that, we will use such well-calculated thresholds and input values
% through this program for guarantees of the reliability and clearness of
% the results is because of our data generation quality or AI settings.
% Extended stating has written in the lower middle, starts from
% "Actually, I am trying...", note of 2026-09-18.