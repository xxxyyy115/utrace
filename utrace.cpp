/*
 * utrace — Signal Driver/Load Tracing Tool
 *
 * Usage:
 *   utrace drive -d <db> -u <usdb> -s <signal> [-t <time>]
 *   utrace load  -d <db> -u <usdb> -s <signal>
 *
 * Options:
 *   -d <path>   Design database path
 *   -u <path>   USDB waveform file
 *   -s <name>   Full hierarchical signal name
 *   -t <time>   Trace time (drive only; enables active trace)
 *   -h          Show help
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include "uvpi.h"
#include "uvpi_trace.h"

using namespace uvpi;

// Get the USDB time scale (e.g., "1ps") via usdbaccess --info.
// Returns the scale string, or empty on failure.
static std::string get_usdb_time_scale(const char* usdb_path) {
    std::string cmd = "usdbaccess -i ";
    cmd += usdb_path;
    cmd += " --info 2>&1";

    std::array<char, 256> buf;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return "";

    while (fgets(buf.data(), buf.size(), pipe.get())) {
        result += buf.data();
    }

    // Parse "time scale: 1ps" line
    auto pos = result.find("time scale:");
    if (pos == std::string::npos) return "";
    pos += 11; // skip "time scale:"
    while (pos < result.size() && result[pos] == ' ') pos++;
    auto end = result.find_first_of(" \n\r\t", pos);
    return result.substr(pos, end - pos);
}

// Parse time unit string to exponent (seconds-based).
// "s" -> 0, "ms" -> -3, "us" -> -6, "ns" -> -9, "ps" -> -12, "fs" -> -15
static int time_unit_exp(const std::string& s) {
    if (s == "s")   return 0;
    if (s == "ms")  return -3;
    if (s == "us")  return -6;
    if (s == "ns")  return -9;
    if (s == "ps")  return -12;
    if (s == "fs")  return -15;
    return 0;
}

// Convert a time value + unit to USDB precision units.
// e.g., convert_time(10, "ns", "1ps") -> 10000
static double convert_time(double value, const std::string& from_unit,
                           const std::string& usdb_scale) {
    // Parse usdb_scale: e.g., "1ps" -> value=1, unit="ps"
    size_t i = 0;
    while (i < usdb_scale.size() && (isdigit(usdb_scale[i]) || usdb_scale[i] == '.'))
        i++;
    double scale_val = std::stod(usdb_scale.substr(0, i));
    std::string scale_unit = usdb_scale.substr(i);

    // Both in seconds: from = value * 10^from_exp, to = scale_val * 10^scale_exp
    // Result = from / to = (value/scale_val) * 10^(from_exp - scale_exp)
    int from_exp = time_unit_exp(from_unit);
    int scale_exp = time_unit_exp(scale_unit);
    int exp_diff = from_exp - scale_exp;

    double ratio = value / scale_val;
    if (exp_diff >= 0) {
        for (int j = 0; j < exp_diff; j++) ratio *= 10.0;
    } else {
        for (int j = 0; j < -exp_diff; j++) ratio /= 10.0;
    }
    return ratio;
}

// Parse the -t argument: "value" or "value unit" (e.g., "10ns", "10 ns", "10").
// Returns true on success, sets out_value and out_unit (empty if no unit given).
static bool parse_time_arg(const char* arg, double& out_value, std::string& out_unit) {
    const char* p = arg;
    // Skip leading spaces
    while (*p == ' ' || *p == '\t') p++;

    // Read numeric part
    char* endp = nullptr;
    out_value = strtod(p, &endp);
    if (endp == p) return false; // no number found

    // Skip spaces between number and unit
    p = endp;
    while (*p == ' ' || *p == '\t') p++;

    // Remaining is the unit (if any)
    out_unit = p;
    return true;
}

static void usage(const char* prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s drive -d <db> -u <usdb> -s <signal> [-t <time>[<unit>]]\n"
        "  %s load  -d <db> -u <usdb> -s <signal>\n"
        "\n"
        "Options:\n"
        "  -d <path>   Design database path\n"
        "  -u <path>   USDB waveform file\n"
        "  -s <name>   Full hierarchical signal name\n"
        "  -t <time>   Trace time for active trace (drive only)\n"
        "              Without unit: value is in USDB time precision units\n"
        "              With unit:    s, ms, us, ns, ps, fs (e.g., -t 10ns)\n"
        "  -h          Show this help\n",
        prog, prog);
}

static const char* active_state_str(DlActiveStateEnum state) {
    switch (state) {
        case DlForceE:      return "Force";
        case DlActiveE:     return "Active";
        case DlPossibleE:   return "Possible";
        case DlInactiveE:   return "Inactive";
        case DlUnanalyzedE: return "Unanalyzed";
        case DlNotAnalyzedE: return "NotAnalyzed";
        default:            return "Unknown";
    }
}

static void print_trace_result(const char* kind, const char* signal,
                               trace_stmt_vec& stmt_vec, int count,
                               bool active_only) {
    if (count == 0) {
        printf("No %ss found for signal '%s'.\n", kind, signal);
        return;
    }

    // Count active items if filtering
    int shown = 0;
    for (int i = 0; i < (int)stmt_vec.size(); i++) {
        if (!active_only || stmt_vec[i]->state() == DlActiveE)
            shown++;
    }

    if (shown == 0) {
        printf("No active %ss found for signal '%s'.\n", kind, signal);
        return;
    }

    if (active_only)
        printf("%s %s (active trace, %d active of %d total)\n",
               kind, signal, shown, count);
    else
        printf("%s %s (%d result%s)\n", kind, signal, count, count > 1 ? "s" : "");

    printf("%-4s  %-50s  %-20s  %-20s  %s\n",
           "#", "statement", "scope", "file:line", "state");
    printf("%-4s  %-50s  %-20s  %-20s  %s\n",
           "----", "--------------------------------------------------",
           "--------------------", "--------------------", "---------");

    int seq = 0;
    for (int i = 0; i < (int)stmt_vec.size(); i++) {
        uvpi_trace_stmt_item* item = stmt_vec[i];

        // In active mode, skip non-active items
        if (active_only && item->state() != DlActiveE)
            continue;

        seq++;
        char loc[256];
        snprintf(loc, sizeof(loc), "%s:%u", item->file_name(), item->file_line());

        printf("%-4d  %-50.50s  %-20.20s  %-20.20s  %s",
               seq, item->name(), item->scope(), loc,
               active_state_str(item->state()));

        if (item->time() >= 0)
            printf("  @%.0f", item->time());
        printf("\n");

        // Print contributor signals (for drivers)
        contr_sig_vec& cv = item->contributor();
        for (int j = 0; j < (int)cv.size(); j++) {
            uvpi_contr_sig_item* c = cv[j];
            printf("      contributor: %-30s  scope=%-15s  value='%s'\n",
                   c->name(), c->scope(), c->value());
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    // Parse subcommand
    const char* subcmd = argv[1];
    if (strcmp(subcmd, "-h") == 0 || strcmp(subcmd, "--help") == 0) {
        usage(argv[0]);
        return 0;
    }

    bool is_drive = (strcmp(subcmd, "drive") == 0);
    bool is_load  = (strcmp(subcmd, "load") == 0);

    if (!is_drive && !is_load) {
        fprintf(stderr, "Error: unknown subcommand '%s'. Use 'drive' or 'load'.\n", subcmd);
        usage(argv[0]);
        return 1;
    }

    // Parse options
    const char* db_path   = nullptr;
    const char* usdb_path = nullptr;
    const char* signal    = nullptr;
    const char* time_str  = nullptr;
    std::string time_buf_combined; // for combining "-t value unit"

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            db_path = argv[++i];
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            usdb_path = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            signal = argv[++i];
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            time_str = argv[++i];
            // Check if next arg is a time unit (s, ms, us, ns, ps, fs)
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                const char* units[] = {"s", "ms", "us", "ns", "ps", "fs"};
                for (auto u : units) {
                    if (strcmp(argv[i + 1], u) == 0) {
                        // Combine "value" + "unit" into a single string
                        time_buf_combined = std::string(time_str) + argv[++i];
                        time_str = time_buf_combined.c_str();
                        break;
                    }
                }
            }
        } else if (strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Error: unknown option '%s'.\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    // Validate required options
    if (!db_path || !usdb_path || !signal) {
        fprintf(stderr, "Error: -d, -u, and -s are all required.\n\n");
        usage(argv[0]);
        return 1;
    }

    if (time_str && is_load) {
        fprintf(stderr, "Warning: -t is ignored for 'load' subcommand.\n");
        time_str = nullptr;
    }

    // Build UVPI argv: pass -d and -u through to uvpi_load_design
    std::vector<char*> uvpi_argv;
    uvpi_argv.push_back(argv[0]);

    char opt_d[] = "-d";
    char opt_u[] = "-u";
    uvpi_argv.push_back(opt_d);
    uvpi_argv.push_back(const_cast<char*>(db_path));
    uvpi_argv.push_back(opt_u);
    uvpi_argv.push_back(const_cast<char*>(usdb_path));

    int uvpi_argc = (int)uvpi_argv.size();

    // Initialize UVPI
    if (!uvpi_init(uvpi_argc, uvpi_argv.data())) {
        fprintf(stderr, "Error: uvpi_init failed.\n");
        return 1;
    }

    // Load design
    if (!uvpi_load_design(uvpi_argc, uvpi_argv.data())) {
        fprintf(stderr, "Error: uvpi_load_design failed (db='%s', usdb='%s').\n",
                db_path, usdb_path);
        uvpi_end();
        return 1;
    }

    // Perform trace
    trace_stmt_vec stmt_vec;
    uvpi_trace_opt opt;
    opt.isCrossHier = true;
    opt.isShowPassThrough = true;

    int count = 0;
    bool active_only = (time_str != nullptr);

    // Convert time_str to USDB precision units:
    // -t value      → pass value directly (already in USDB precision units)
    // -t value unit → convert to USDB precision units (e.g., -t 10ns → 10000 for 1ps USDB)
    std::string time_buf;
    if (time_str) {
        std::string usdb_scale = get_usdb_time_scale(usdb_path);
        if (usdb_scale.empty()) {
            fprintf(stderr, "Warning: could not determine USDB time scale, "
                    "passing time value as-is.\n");
        } else {
            fprintf(stderr, "USDB time scale: %s\n", usdb_scale.c_str());
        }

        double user_value = 0;
        std::string user_unit;
        if (!parse_time_arg(time_str, user_value, user_unit)) {
            fprintf(stderr, "Error: invalid time value '%s'.\n", time_str);
            uvpi_end();
            return 1;
        }

        double precision_value;
        if (user_unit.empty() || usdb_scale.empty()) {
            // No unit given or no USDB scale: pass value directly
            precision_value = user_value;
            fprintf(stderr, "Time: %g (in USDB precision units)\n", precision_value);
        } else {
            // Convert from user unit to USDB precision units
            precision_value = convert_time(user_value, user_unit, usdb_scale);
            fprintf(stderr, "Time: %g%s → %g (in USDB precision units)\n",
                    user_value, user_unit.c_str(), precision_value);
        }

        char buf[64];
        snprintf(buf, sizeof(buf), "%.0f", precision_value);
        time_buf = buf;
        time_str = time_buf.c_str();
    }

    if (is_drive) {
        count = uvpi_trace_driver(signal, stmt_vec,
                                  time_str, opt);
        print_trace_result("DRIVER", signal, stmt_vec, count, active_only);
    } else {
        count = uvpi_trace_load(signal, stmt_vec, opt);
        print_trace_result("LOAD", signal, stmt_vec, count, false);
    }

    // Cleanup
    uvpi_release_trace_result(stmt_vec);
    uvpi_release_all_handles();
    uvpi_end();

    return (count > 0) ? 0 : 1;
}
