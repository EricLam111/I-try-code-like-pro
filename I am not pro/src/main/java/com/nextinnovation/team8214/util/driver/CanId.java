package com.nextinnovation.team8214.util.driver;

import com.ctre.phoenix6.CANBus;

public record CanId(int id, CANBus bus) {
  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }

    if (!(o instanceof CanId canId)) {
      return false;
    }

    return id == canId.id && bus.equals(canId.bus);
  }
}
