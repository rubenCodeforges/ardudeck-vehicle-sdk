#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "conform.h"

#define VERSION "0.1.0"

static const char *USAGE =
  "ardudeck-conform " VERSION "\n"
  "\n"
  "Test whether a vehicle does what it says it does.\n"
  "\n"
  "  ardudeck-conform --udp 14550\n"
  "  ardudeck-conform --udp 192.168.4.1:14550\n"
  "  ardudeck-conform --serial /dev/ttyUSB0:57600\n"
  "\n"
  "Options\n"
  "  --udp PORT | HOST:PORT     listen, and answer whoever is heard\n"
  "  --serial DEVICE:BAUD       9600 to 230400\n"
  "  --json                     machine readable, for CI\n"
  "  --allow-mission-write      test missions on a vehicle that cannot read one\n"
  "                             back. This REPLACES the stored plan.\n"
  "  --verbose                  say what is happening as it happens\n"
  "  -h, --help                 this\n"
  "\n"
  "Safety\n"
  "  This never arms, never disarms and never changes flight mode. It will not\n"
  "  start a calibration that declares motors will spin. It reads the stored\n"
  "  mission first and puts it back afterwards, and when it cannot read one it\n"
  "  skips the mission tests rather than overwriting a plan.\n"
  "\n"
  "Exit code is 0 when every declared capability passes, 1 otherwise.\n";

static const char *word_for(result_t r) {
  switch (r) {
    case R_PASS: return "PASS";
    case R_WARN: return "WARN";
    case R_FAIL: return "FAIL";
    default:     return "SKIP";
  }
}

static const char *colour_for(result_t r, bool tty) {
  if (!tty) return "";
  switch (r) {
    case R_PASS: return "\033[32m";
    case R_WARN: return "\033[33m";
    case R_FAIL: return "\033[31m";
    default:     return "\033[90m";
  }
}

static void print_table(const rung_t *rungs, int n, const vehicle_t *v, bool tty) {
  printf("\n");
  if (v->have_manifest) {
    printf("%s %s  fw %s  profile %u\n", v->vendor, v->model, v->firmware,
           v->profile_version);
  } else {
    printf("unidentified vehicle, system %u\n", v->sysid);
  }
  printf("\n");

  for (int i = 0; i < n; i++) {
    const rung_t *r = &rungs[i];
    printf("%s%-4s%s  %-7s %-13s %s\n", colour_for(r->result, tty),
           word_for(r->result), tty ? "\033[0m" : "", r->rung, r->name, r->summary);
    for (int d = 0; d < r->detail_count; d++) {
      printf("                            -> %s\n", r->detail[d]);
    }
  }
}

static void print_verdict(const rung_t *rungs, int n) {
  int passed = 0, tested = 0, failed = 0;
  for (int i = 0; i < n; i++) {
    if (rungs[i].result == R_SKIP) continue;
    tested++;
    if (rungs[i].result == R_FAIL) failed++;
    else passed++;
  }

  printf("\n%d of %d tested, %d failing.\n", passed, tested, failed);

  printf("ArduDeck will enable:");
  bool any = false;
  for (int i = 0; i < n; i++) {
    if (rungs[i].result == R_FAIL || rungs[i].result == R_SKIP) continue;
    if (!strcmp(rungs[i].name, "link")) continue; /* behaviour in flight, not a screen */
    printf("%s %s", any ? "," : "", rungs[i].name);
    any = true;
  }
  if (!any) printf(" nothing");
  printf("\n");

  for (int i = 0; i < n; i++) {
    if (rungs[i].result == R_FAIL) {
      if (!strcmp(rungs[i].name, "link")) {
        printf("the link check failed, which is a flight safety problem rather than a "
               "hidden screen.\n");
      } else {
        printf("%s stays hidden until it passes.\n", rungs[i].name);
      }
    }
  }
}

static void print_json(const rung_t *rungs, int n, const vehicle_t *v) {
  printf("{\n");
  printf("  \"vendor\": \"%s\",\n", v->vendor);
  printf("  \"model\": \"%s\",\n", v->model);
  printf("  \"firmware\": \"%s\",\n", v->firmware);
  printf("  \"profile\": %u,\n", v->profile_version);
  printf("  \"rungs\": [\n");
  for (int i = 0; i < n; i++) {
    printf("    { \"rung\": \"%s\", \"name\": \"%s\", \"result\": \"%s\", "
           "\"summary\": \"%s\", \"findings\": [",
           rungs[i].rung, rungs[i].name, word_for(rungs[i].result), rungs[i].summary);
    for (int d = 0; d < rungs[i].detail_count; d++) {
      printf("%s\"%s\"", d ? ", " : "", rungs[i].detail[d]);
    }
    printf("] }%s\n", i + 1 < n ? "," : "");
  }
  printf("  ]\n}\n");
}

int main(int argc, char **argv) {
  const char *udp = NULL;
  const char *serial = NULL;
  bool json = false, verbose = false, allow_write = false;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--udp") && i + 1 < argc) udp = argv[++i];
    else if (!strcmp(argv[i], "--serial") && i + 1 < argc) serial = argv[++i];
    else if (!strcmp(argv[i], "--json")) json = true;
    else if (!strcmp(argv[i], "--verbose")) verbose = true;
    else if (!strcmp(argv[i], "--allow-mission-write")) allow_write = true;
    else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
      fputs(USAGE, stdout);
      return 0;
    } else {
      fprintf(stderr, "unknown argument '%s'\n\n%s", argv[i], USAGE);
      return 2;
    }
  }

  if (!udp && !serial) {
    fputs(USAGE, stderr);
    return 2;
  }

  char err[256] = "";
  session_t s;
  memset(&s, 0, sizeof s);
  s.sysid = 255;
  s.compid = 190;
  s.verbose = verbose;
  s.allow_mission_write = allow_write;
  s.link = udp ? link_open_udp(udp, err, sizeof err)
               : link_open_serial(serial, err, sizeof err);
  if (!s.link) {
    fprintf(stderr, "cannot open the link: %s\n", err);
    return 2;
  }
  mav_reset(&s.parser);

  if (!json) {
    printf("listening on %s%s\n", udp ? "udp " : "serial ", udp ? udp : serial);
  }

  rung_t rungs[7];
  memset(rungs, 0, sizeof rungs);
  rungs[0].rung = "rung 0"; rungs[0].name = "position";
  rungs[1].rung = "rung 1"; rungs[1].name = "identity";
  rungs[2].rung = "rung 2"; rungs[2].name = "parameters";
  rungs[3].rung = "rung 3"; rungs[3].name = "missions";
  rungs[4].rung = "rung 4"; rungs[4].name = "commands";
  rungs[5].rung = "extra";  rungs[5].name = "calibration";
  rungs[6].rung = "extra";  rungs[6].name = "link";

  check_rung0(&s, &rungs[0]);

  /* Nothing above rung 0 can be judged without knowing what was claimed, and a silent
     link is a link problem rather than a conformance result. */
  if (rungs[0].result == R_FAIL && s.v.heartbeats == 0) {
    for (int i = 1; i < 7; i++) {
      rungs[i].result = R_SKIP;
      snprintf(rungs[i].summary, sizeof rungs[i].summary, "no link");
    }
  } else {
    check_rung1(&s, &rungs[1]);
    check_rung2(&s, &rungs[2]);
    check_rung3(&s, &rungs[3]);
    check_rung4(&s, &rungs[4]);
    check_calibration(&s, &rungs[5]);
    check_link(&s, &rungs[6]);
  }

  bool tty = getenv("NO_COLOR") == NULL;
  if (json) {
    print_json(rungs, 7, &s.v);
  } else {
    print_table(rungs, 7, &s.v, tty);
    print_verdict(rungs, 7);
  }

  link_close(s.link);

  for (int i = 0; i < 6; i++) {
    if (rungs[i].result == R_FAIL) return 1;
  }
  return 0;
}
