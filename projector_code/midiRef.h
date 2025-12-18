////midi notes sharps or flats/////
int shFl[] = { 37, 39, 42, 44, 46, 49, 51, 54, 56, 58, 61, 63, 66, 68, 70, 73, 75, 78, 80, 82, 85, 87, 90, 92, 94 };

int shFlCount = 25;

int naturals[] = { 36, 38, 40, 41, 43, 45, 47, 48, 50, 52, 53, 55, 57, 59, 60, 62, 64, 65, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 84, 86, 88, 89, 91, 93, 95, 96 };

int naturalsCount = 36;


int keyflag = 0;
int midiCurrentKeys = 0;
int prevmidiCurrentKeys = 0;
int midiCurrentChannel = 0;
int currentPitchBendVal = 0;
int highestNote = 0;
int highestNoteAdj = 0;
int lowestNote = 0;
int lowestNoteAdj = 0;

int midiAdjustedPitchBendVal = 0;
int pitchBendReframe = 0;
int musicMode = 0;
int CC1ProjSpeed;
int CC2ProjBlades;
int CC3ProjBright;
int moveStatus;

int midiKeyCommand = 0;
int moveCMD = 0;
int midiSong = 0;
int MSFStatus = 0;
int sendCommand = 0;

int averageKeysLoc = 0;

float midiNotesPlayed[10];
int midiNotesPlayedIndex = 0;

int moveRecvd = 0;
int noteRecvd = 0;
int moveHalt = 0;
int singleFrameOperation = 0;

int shFlKeysPlayed = 0;
int naturalsKeysPlayed = 0;
int firstTimeF = 0;
int firstTimeR = 0;

int calculateValue = 0;

elapsedMillis midiKeyTimer;
elapsedMillis SingleTimer;

struct MyMIDI_Callbacks : MIDI_Callbacks {



  // Callback for channel messages (notes, control change, pitch bend, etc.).
  void onChannelMessage(MIDI_Interface &, ChannelMessage msg) override {
    calculateValue = 1;
    midiKeyTimer = 0;
    MSFStatus = 0;


    auto type = msg.getMessageType();
    if (type == msg.NoteOn || type == msg.NoteOff) {
      onNoteMessage(msg);
    } else if (type == msg.PitchBend) {
      onPitchBendMessage(msg);
    }
  }
  void onNoteMessage(ChannelMessage msg) {

    auto type = msg.getMessageType();
    auto channel = msg.getChannel().getOneBased();
    auto note = msg.getData1();
    auto velocity = msg.getData2();



    midiCurrentChannel = channel;

        if (midiNotesPlayedIndex <= 9) {
          midiNotesPlayedIndex = midiNotesPlayedIndex + 1;
        } else {
          midiNotesPlayedIndex = 0;
        }

    if (velocity == 64){
              sendCommand = 0;
      if (midiCurrentKeys != 0) {
        midiCurrentKeys = midiCurrentKeys + 1;


        midiNotesPlayed[midiNotesPlayedIndex] = note;


      } else if (midiCurrentKeys == 0) {
        midiSong = 1;
        midiCurrentKeys = midiCurrentKeys + 1;
        midiNotesPlayedIndex = 0;
        midiNotesPlayed[midiNotesPlayedIndex] = note;
      }
    }
    if (velocity == 0)
      midiCurrentKeys = midiCurrentKeys - 1;
    if (midiCurrentKeys == 0) {
      memset(midiNotesPlayed, 0, sizeof(midiNotesPlayed));
    }
  }



  void onPitchBendMessage(ChannelMessage msg) {
    auto type = msg.getMessageType();
    auto channel = msg.getChannel().getOneBased();
    auto PitchBend = msg.getMessageType();

    currentPitchBendVal = msg.getData14bit();
    // if (debugEnable == 1)
    //     Serial << channel << " pitchbend " << currentPitchBendVal << endl;
        sendCommand = 0;
  }


} callback;  // Instantiate a callback
