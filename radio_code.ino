void CheckRadio() {
  if (radio.reciveDone()) {
    char radioCommand[63];
    int commandLength = 0;

    for (int i = 0; i < radio.DATALEN) {
      if (radio.DATA[i] !== ' ') {
        radioCommand[i] = radio.DATA[i];
      } else {
        radioCommand[i] = '\0'
        commandLength = i;
        break;
      }
      
    }

    if(strcmp(radioCommand, "GET") == 0) {
      
    }


    if (radio.ACKRequested()) {
      radio.sendACK();
    }
  }
}

/*================SEAGLIDE=RADIO=COMMANDS================*/
/*GET <VALUE_NAME : can be one of these values>
/*   - PRESSURE : <int : 0-1023>
/*   - CURRENT_HEADDING : <float : 0-360>
/*   - TARGET_HEADDING : <float : 0-360>
/*   - ENGINE_STATE : <String : one of listed values>
/*     - ENGINE_RISING
/*     - ENGINE_DIVEING
/*     - ENGINE_IDLE_RISING
/*     - ENGINE_IDLE_DIVING
/*
/*SET <VALUE_NAME : can be one of these values>
/*   - TARGET_HEADING : <float : 0-360>
/*
/*RUN <FUNCTION_NAME : String : can be one of these values>
/*   - "CENTER"
/*   - "BURP"
/*   - ""
/*
/*CONTINUE <void>
/*   - tells the seaglide to ignore the countdown timer and just continue
/*
/*PAUSE <void>
/*   - returns the current pause state <true | false>
/*
/*PAUSE <boolean>
/*   - sets the current pause state + sends ACK
/*
/*
*/
