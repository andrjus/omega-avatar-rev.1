#include "core/robosd_cell.c"
#include "core/robosd_string.c"
#include "core/robosd_vartable.c"
#include "core/robosd_cvartable.cpp"
#include "freemaster/robosd_fm.cpp"
#include "net/switch/robosd_proto_switch.cpp"
#include "terminal/robosd_termo.cpp"
#include "terminal/robosd_termo_vartable_cmd.cpp"



#include "mexo/mexo.c"
#include "mexo/mexo.cpp"
#include "mexo/net/flow/dev.flow.proto.c"
#include "mexo/net/adapters/flow_port.c"
#include "mexo/net/adapters/flow_msg_port.c"
#include "mexo/machine/power/generator.proto.stub.c"
#include "mexo/mexo_signal.c"
#include "mexo/mexo_sin.c"
#include "mexo/mexo_filter.c"
#include "mexo/mexo_regulator.c"
//#include "mexo/net/flow/dev.flow.hand.proto.c"
#include "mexo/net/flow/actuator.flow.proto.c"

#include "net/serial/robosd_serial.cpp"

#ifdef IMITATION_MODEL
#include "net/serial/platform/win/win_com.cpp"
#endif

