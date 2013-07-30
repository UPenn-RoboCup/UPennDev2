#include "spnav.h"

int spnav_parse(structSpnav *ud, int len) {
  /* parse event type */     
  int event_type = ud->buf[0] - 1;

  switch (event_type) {
    case SPNAV_EVENT_EMPTY:
//      fprintf(stdout, "empty event\n");
      break;
    case SPNAV_EVENT_ROTATE:
      fprintf(stdout, "rotate event\n");
      break;
    case SPNAV_EVENT_TRANSLATE:
      fprintf(stdout, "translate event\n");
      break;
    case SPNAV_EVENT_BUTTON:
      fprintf(stdout, "button event\n");
      break;
    default:
      break;
  }

  return 1;
}
