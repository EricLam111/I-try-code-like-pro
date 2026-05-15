package com.nextinnovation.team8214.util.driver;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public record DoubleSolenoidId(PneumaticsModuleType moduleType, int forward, int reverse) {}
