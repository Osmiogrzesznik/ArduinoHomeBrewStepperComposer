#include <TimerEvent.h>
#include <AccelStepper.h>

#define d(x) \
    ;        \
    Serial.print(x)
#define dln(x) Serial.println(x)
#define er(no, x) \
    d("error #"); \
    d(no);        \
    d(":");       \
    d(x);         \
    dln("PP:" + PP);
#define l(x)      \
    d(#x);        \
    d(":(log):"); \
    dln(x);

#define MAX_max_speed = 4000
int lastenteredpostarget;
int a;
bool deb = false;
unsigned long whenStartedLastMove = 0;
unsigned long whenArrivedAtLastPosition = 0;
bool arrivedAtLastposition = true;
bool programRunning = false;
unsigned long programLastStarted = 0;
unsigned int timeToAchieve;
TimerEvent timerOne;

typedef enum
{
    OP_MODE_TEACH,
    OP_MODE_REMOTE,
    OP_MODE_RUNNINGPRG,
    OP_MODE_STEP // repeat with offset (setsCurrentpostoZero)
} OperationMode;

typedef enum
{
    ONCE,
    ALTERNATE,              //
    ALTERNATE_REVERSE_FAST, // alternate but come back fast
    REPEAT_FORWARD_RESET_POS,
} Prog_Exec_Mode;

OperationMode op_mode = OperationMode::OP_MODE_TEACH;
Prog_Exec_Mode prog_mode = Prog_Exec_Mode::ONCE;
unsigned long prev_ms;
float timeFactor = 1;
int max_speed = 16000;
int speed = 1000;
unsigned long acceleration = 10000;
unsigned long updatedLast = 0;
unsigned long plot_update_interval = 1000;
// pos,time to achieve

/*
pos
acc
max_speed
_______________________________
last_direction  --- to determine if should decelerate if next ProgKeyf is in the same direction not stopping at each pos
                --- check in middle of movement if max_speed is reached (somehow)

*/
int prg_step = 1; // if negative program is run in reverse
const int PRG_MAX_SIZE = 8;
byte prg_LAST = 7;   // last keyframe considered part of program
byte t_prg_LAST = 0; // last keyframe considered part of program
long prg_pos[PRG_MAX_SIZE] = {200, 0, 400, 0, 800, 0, 1600, 0};
unsigned int prg_acc[PRG_MAX_SIZE] = {10, 16000, 100, 16000, 1000, 16000, 10000, 16000};
int prg_max[PRG_MAX_SIZE] = {max_speed, max_speed, max_speed, max_speed, max_speed, max_speed, max_speed, max_speed};
long t_prg_pos[PRG_MAX_SIZE] = {200, 0, 400, 0, 800, 0, 1600, 0};
unsigned int t_prg_acc[PRG_MAX_SIZE] = {10, 16000, 100, 16000, 1000, 16000, 10000, 16000};
int t_prg_max[PRG_MAX_SIZE] = {max_speed, max_speed, max_speed, max_speed, max_speed, max_speed, max_speed, max_speed};
byte TP = 0;
byte PP = 0;
byte PR = 0;

AccelStepper stepper(AccelStepper::FULL2WIRE, 2, 5);
// const AccelStepperBo motors[1] = {WheelsStepper};

unsigned long timeMultiplied()
{
    return millis() * abs(timeFactor);
}
void onProgramEndReached()
{
    dln("___________________PROGRAM_END_REACHED____________");
    switch (prog_mode)
    {
    case ONCE:
        /* code */
        program_stop();
        break;
    case ALTERNATE:
        /* code */
        prg_step = -prg_step;
        break;
    case ALTERNATE_REVERSE_FAST:
        /* code */
        break;
    case REPEAT_FORWARD_RESET_POS:
        /* code */
        break;
    default:
        break;
    }

    //if mode repeat: program_start()
    //if mode repeat:
    //if mode Alternate: prg_step = -prg_step;program_start()
}

void onProgramStartReached()
{
    dln("___________________PROGRAM_START_REACHED____________");
    switch (prog_mode)
    {
    case ONCE:
        /* code */
        program_stop();
        break;
    case ALTERNATE:
        /* code */
        prg_step = -prg_step;
        break;
    case ALTERNATE_REVERSE_FAST:
        /* code */
        break;
    case REPEAT_FORWARD_RESET_POS:
        /* code */
        break;
    default:
        break;
    }
    //if mode repeat: program_start()
    //if mode repeat:
    //if mode Alternate: prg_step = -prg_step;program_start()
}

void program_update()
{

    if (!programRunning)
    {
        return;
    }

    // dln("prog update");
    // if timeKeyf is smaller than current exec time

    if (stepper.distanceToGo() != 0UL)
    {
        return;
        //TODO: later turn error in one byte feedback to display that this program
        //       is physically impossible to nail - too fast,
        // but first figure out speed to steps ratio.
    }

    unsigned long howLongLastKeyfTook = whenArrivedAtLastPosition - whenStartedLastMove;
    // d("howLongLastKeyfTook:");
    // dln(howLongLastKeyfTook);
    l(howLongLastKeyfTook);

    // time for next keyframe

    if (PP == PRG_MAX_SIZE && prg_step > 0)
    {
        onProgramEndReached();
    }
    else if (PP == 0 && prg_step < 0)
    {
        onProgramStartReached();
    }
    else
    {
        dln("_______finished Keyframe:");
        l(PP);
        l(timeMultiplied());
        //okay time to start next move but did the motor reached
        //its destination?
        long distToGo = stepper.distanceToGo();
        l(distToGo);

        //NEXT KEYFRAME
        if (op_mode == OP_MODE_STEP)
        {
            PP = PP + prg_step;
            programRunning = false;
        }
        else
        {
            programRunning = true;
            PP = PP + prg_step;
            exec_prg_keyframe(PP);
        }

        // prg[pp+1]
        //if it is move at max speed
        // ____________________________________
        // function EXEC KEYFRAME
        // _____________________________
    }
}

void exec_prg_keyframe(byte P)
{
    //if op_mode = step_mode then
    //IF in reverse mode ALTERNATE_REVERSE_FAST
    //IF IN REVERSE - SHOULD IT READ CUR POS BUT PREV ACCEL etc.? acc[p] but pos[p-1]
    unsigned long targetPos = prg_pos[P];
    stepper.setAcceleration(prg_acc[P]);
    //if acc = 0 and prev direction is same as next direction then no need to decelerate
    // maybe: array of type of keyframe transition- with or without decel
    //then switch to runSpeedToPosition
    //
    stepper.setMaxSpeed(prg_max[P]);
    stepper.moveTo(prg_pos[P]);
    d("_______starting Keyframe:");
    l(P);
    l(targetPos);
    l(stepper.speed());
    l(stepper.distanceToGo());
    whenStartedLastMove = timeMultiplied();
}

void program_start()
{
    op_mode = OperationMode::OP_MODE_RUNNINGPRG;
    PP = 0;
    programLastStarted = timeMultiplied();
    programRunning = true;
    exec_prg_keyframe(PP);
}

void program_stop()
{
    programRunning = false;
    PP = 0;
    op_mode = OP_MODE_REMOTE;
}

void setup()
{
    Serial.begin(9600);
    stepper.setMaxSpeed(max_speed);

    stepper.setAcceleration(acceleration);

    deb = false;
    dln("Starting test...");
    dln("a,lastenteredpostarget");
}

void cycle2()
{
    loop();
}
void change_op_mode(char c)
{
    dln(c);
    dln(int(c));
    dln("changing mode:");
    switch (c)
    {
    case 'T':
        op_mode = OperationMode::OP_MODE_TEACH;
        l(op_mode);
        break;

    case 'P':
        //this UNNEDED !!!!!!!!!!!!!!!!!!!!!!!!!!
        // AUTOMATICALLY WILL ENTER this mode on start
        op_mode = OperationMode::OP_MODE_RUNNINGPRG;

        l(op_mode);

        break;
    case 'R':
        op_mode = OperationMode::OP_MODE_TEACH;
        l(op_mode);

        break;
    case 'S':
        op_mode = OperationMode::OP_MODE_STEP;
        l(op_mode);

        break;
    }
}

void insertPosKeyframe()
{

    t_prg_pos[TP] = stepper.currentPosition();

    t_prg_acc[TP] = acceleration;
    t_prg_max[TP] = max_speed;

    if (TP >= PRG_MAX_SIZE)
    {
        er(1, "max program capacity");
    }
    else
    {
        t_prg_LAST = TP;
        TP += prg_step;
    }
}

void deleteLastKeyframe()
{
    er(0, "not implemented");
}

void program_receive()
{
    // Pp100a100m100p100a100m100p100a100m100
}

void program_export()
{
    //separate Serial print
}

void saveTeachAsProg()
{
    if (TP == 0)
    {
        er(2, "nothing to save");
        return;
    }
    for (i = 0; i < PRG_MAX_SIZE; i++)
    {
        prg_acc[i] = t_prg_acc[i];
        prg_max[i] = t_prg_max[i];
        prg_pos[i] = t_prg_pos[i];
    }
    prg_LAST = t_prg_LAST;
    d("program saved");
    TP = 0;
}

void loop()
{
    if (Serial.available() > 0)
    {
        delay(1);
        char c = Serial.read();
        dln(c);
        dln(int(c));

        switch (c)

        {
        //SsmatTedb+-
        case 10:
            break;
        case 'M':
            c = Serial.read();
            change_op_mode(c);
            break;
        case 'f':
            //insert keyframe/remember position
            insertPosKeyframe();
            break;
        case 'g':
            deleteLastKeyframe();
            break;
        case 'T':
            saveTeachAsProg();
            break;
        case 'A':

            acceleration = Serial.parseInt();
            if (acceleration < 1)
            {
                acceleration = 1;
            }
            acceleration = acceleration;
            stepper.setAcceleration(acceleration);
            l(acceleration);
            break;
        case 't':
            timeFactor = Serial.parseInt();
            l(timeFactor);
            break;
        case 'e':
            plot_update_interval = Serial.parseInt();
            l(plot_update_interval);
            break;
        case '>':
            op_mode = OP_MODE_STEP;
            programRunning = true;
            break;
        case 'S':
            max_speed = Serial.parseInt();
            stepper.setMaxSpeed(max_speed);
            l(max_speed);
            break;
        case 's':
            speed = Serial.parseInt();
            stepper.setSpeed(speed);
            l(speed);
            break;
        case 'm':

            lastenteredpostarget = Serial.parseInt();
            stepper.setSpeed(max_speed);
            stepper.setMaxSpeed(max_speed);
            stepper.moveTo(lastenteredpostarget);
            //move

            break;
        case 'a':
            program_start();
            /*goid idea: when resetting program separate option to go back in reverse from current pp
                in fast mode, to quickly retry the shot*/
            // mytimeline.start();
            break;
        case 'b':
            program_stop();
            // mytimeline.stop();
            break;
        case '+':
            plot_update_interval = plot_update_interval * 2;
            timeFactor = timeFactor * 2;
            l(plot_update_interval);
            l(timeFactor);
            break;
        case '-':
            plot_update_interval = plot_update_interval / 2;
            timeFactor = timeFactor / 2;
            l(plot_update_interval);
            l(timeFactor);
            break;
        case 'P':
            program_receive();
            break;
        case 'X':
            program_export();
            break;
        case 'd':
        {

            dln("toggling debugging");
            deb = !deb;
            dln(deb ? "yesdeb" : "nodeb");
            break;
        }
        default:
            dln("not recog char");
            break;

            // d("debugging");
        }
    }

    bool runningtoPosition = stepper.run();

    if (timeMultiplied() > prev_ms + plot_update_interval)
    {
        if (deb)
        {
            d(millis());
            d(", ");

            d(timeMultiplied());
            d(", ");
            d(stepper.currentPosition());
            d(", ");
            d(lastenteredpostarget);
            d(", ");
            d(stepper.speed());
            dln();
        }
        prev_ms = timeMultiplied();
    }
    // timerOne.update();

    if (runningtoPosition)
    {
        arrivedAtLastposition = false;
    }

    else if (!runningtoPosition && !arrivedAtLastposition)
    {
        whenArrivedAtLastPosition = timeMultiplied();
        arrivedAtLastposition = true;
        d("reached position ");
        dln(stepper.currentPosition());
    }
    program_update();

    // mytimeline.update();
}
