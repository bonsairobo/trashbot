#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

#include "zhash.h"
#include "zarray.h"
#include "getopt.h"

#define GOO_BOOL_TYPE 1
#define GOO_STRING_TYPE 2

typedef struct getopt_option getopt_option_t;

struct getopt_option {
	char *sname;
	char *lname;
	char *svalue;

	char *help;
	int type;

	int spacer;

    int was_specified;
};

struct getopt {
    zhash_t  *lopts;
    zhash_t  *sopts;
    zarray_t *extraargs;
    zarray_t *options;
};

getopt_t *
getopt_create (void)
{
    getopt_t *gopt = calloc (1, sizeof(*gopt));

    gopt->lopts     = zhash_create (sizeof(char*), sizeof(getopt_option_t*), zhash_str_hash, zhash_str_equals);
    gopt->sopts     = zhash_create (sizeof(char*), sizeof(getopt_option_t*), zhash_str_hash, zhash_str_equals);
    gopt->options   = zarray_create (sizeof(getopt_option_t*));
    gopt->extraargs = zarray_create (sizeof(char*));

    return gopt;
}

void
getopt_option_destroy (getopt_option_t *goo)
{
    free (goo->sname);
    free (goo->lname);
    free (goo->svalue);
    free (goo->help);
    free (goo);
}

void
getopt_destroy (getopt_t *gopt)
{
    // free the extra arguments and container
    zarray_vmap (gopt->extraargs, free);
    zarray_destroy (gopt->extraargs);

    // deep free of the getopt_option structs. Also frees key/values, so
    // after this loop, hash tables will no longer work
    zarray_vmap (gopt->options, getopt_option_destroy);
    zarray_destroy (gopt->options);

    // free tables
    zhash_destroy (gopt->lopts);
    zhash_destroy (gopt->sopts);

    free (gopt);
}

static void
getopt_modify_string (char **str, char *newvalue)
{
    char *old = *str;
    *str = newvalue;
    if (old != NULL)
        free (old);
}

// returns 1 if no error
int
getopt_parse (getopt_t *gopt, int argc, char *argv[], int showErrors)
{
    int okay = 1;
    zarray_t *toks = zarray_create (sizeof(char*));

    // take the input stream and chop it up into tokens
    for (int i = 1; i < argc; i++) {
        char *arg = strdup (argv[i]);

        if (arg[0] != '-') {
            // if this isn't an option, put the whole thing in the args list.
            zarray_add (toks, &arg);

        }
        else {
            // this is an option. It could be a flag (like -v), or an option
            // with arguments (--file=foobar.txt).
            char *eq = strstr (arg, "=");

            // no equal sign? Push the whole thing.
            if (eq == NULL) {
                zarray_add (toks, &arg);
            }
            else {
                // there was an equal sign. Push the part
                // before and after the equal sign
                char *val = strdup (&eq[1]);
                eq[0] = 0;
                zarray_add (toks, &arg);

                // if the part after the equal sign is
                // enclosed by quotation marks, strip them.
                if (val[0]=='\"') {
                    int last = strlen (val) - 1;
                    if (val[last]=='\"')
                        val[last] = 0;
                    char *valclean = strdup (&val[1]);
                    zarray_add (toks, &valclean);
                    free (val);
                }
                else
                    zarray_add (toks, &val);
            }
        }
    }

    // now loop over the elements and evaluate the arguments
    unsigned int i = 0;

    char *tok = NULL;

    while (i < zarray_size (toks)) {

        // rather than free statement throughout this while loop
        if (tok != NULL)
            free (tok);

        zarray_get (toks, i, &tok);

        if (0==strncmp (tok,"--", 2)) {
            char *optname = &tok[2];
            getopt_option_t *goo = NULL;
            zhash_get (gopt->lopts, &optname, &goo);
            if (goo == NULL) {
                okay = 0;
                if (showErrors)
                    printf ("Unknown option --%s\n", optname);
                i++;
                continue;
            }

            goo->was_specified = 1;

            if (goo->type == GOO_BOOL_TYPE) {
                if ((i+1) < zarray_size (toks)) {
                    char *val = NULL;
                    zarray_get (toks, i+1, &val);

                    if (0==strcmp (val,"true")) {
                        i+=2;
                        getopt_modify_string (&goo->svalue, val);
                        continue;
                    }
                    if (0==strcmp (val,"false")) {
                        i+=2;
                        getopt_modify_string (&goo->svalue, val);
                        continue;
                    }
                }
                getopt_modify_string (&goo->svalue, strdup("true"));
                i++;
                continue;
            }

            if (goo->type == GOO_STRING_TYPE) {
                // TODO: check whether next argument is an option, denoting missing argument
                if ((i+1) < zarray_size (toks)) {
                    char *val = NULL;
                    zarray_get (toks, i+1, &val);
                    i+=2;
                    getopt_modify_string (&goo->svalue, val);
                    continue;
                }

                okay = 0;
                if (showErrors)
                    printf ("Option %s requires a string argument.\n",optname);
            }
        }

        if (0==strncmp (tok,"-",1) && strncmp (tok,"--",2)) {
            int len = strlen (tok);
            int pos;
            for (pos = 1; pos < len; pos++) {
                char sopt[2];
                sopt[0] = tok[pos];
                sopt[1] = 0;
                char *sopt_ptr = (char*) &sopt;
                getopt_option_t *goo = NULL;
                zhash_get (gopt->sopts, &sopt_ptr, &goo);

                if (goo==NULL) {
                    // is the argument a numerical literal that happens to be negative?
                    if (pos==1 && isdigit (tok[pos])) {
                        zarray_add (gopt->extraargs, &tok);
                        tok = NULL;
                        break;
                    }
                    else {
                        okay = 0;
                        if (showErrors)
                            printf ("Unknown option -%c\n", tok[pos]);
                        i++;
                        continue;
                    }
                }

                goo->was_specified = 1;

                if (goo->type == GOO_BOOL_TYPE) {
                    getopt_modify_string (&goo->svalue, strdup("true"));
                    continue;
                }

                if (goo->type == GOO_STRING_TYPE) {
                    if ((i+1) < zarray_size (toks)) {
                        char *val = NULL;
                        zarray_get (toks, i+1, &val);
                        // TODO: allow negative numerical values for short-name options ?
                        if (val[0]=='-')
                        {
                            okay = 0;
                            if (showErrors)
                                printf ("Ran out of arguments for option block %s\n", tok);
                        }
                        i++;
                        getopt_modify_string (&goo->svalue, val);
                        continue;
                    }

                    okay = 0;
                    if (showErrors)
                        printf ("Option -%c requires a string argument.\n", tok[pos]);
                }
            }
            i++;
            continue;
        }

        // it's not an option-- it's an argument.
        zarray_add (gopt->extraargs, &tok);
        tok = NULL;
        i++;
    }
    if (tok != NULL)
        free (tok);

    zarray_destroy (toks);

    return okay;
}

void
getopt_add_spacer (getopt_t *gopt, const char *s)
{
    getopt_option_t *goo = calloc (1, sizeof(*goo));
    goo->spacer = 1;
    goo->help = strdup (s);
    zarray_add (gopt->options, &goo);
}

void
getopt_add_bool (getopt_t *gopt, char sopt, const char *lname, int def, const char *help)
{
    char sname[2];
    sname[0] = sopt;
    sname[1] = 0;
    char *sname_ptr = (char*) &sname;

    if (strlen (lname) < 1) { // must have long name
        fprintf (stderr, "getopt_add_bool(): must supply option name\n");
        exit (EXIT_FAILURE);
    }

    if (sopt == '-') { // short name cannot be '-' (no way to reference)
        fprintf (stderr, "getopt_add_bool(): invalid option character: '%c'\n", sopt);
        exit (EXIT_FAILURE);
    }

    if (zhash_contains (gopt->lopts, &lname)) {
        fprintf (stderr, "getopt_add_bool(): duplicate option name: --%s\n", lname);
        exit (EXIT_FAILURE);
    }

    if (sopt != '\0' && zhash_contains (gopt->sopts, &sname_ptr)) {
        fprintf (stderr, "getopt_add_bool(): duplicate option: -%s ('%s')\n", sname, lname);
        exit (EXIT_FAILURE);
    }

    getopt_option_t *goo = calloc (1, sizeof(*goo));
    goo->sname = strdup (sname);
    goo->lname = strdup (lname);
    goo->svalue = strdup (def ? "true" : "false");
    goo->type = GOO_BOOL_TYPE;
    goo->help = strdup (help);

    zhash_put (gopt->lopts, &goo->lname, &goo, NULL, NULL);
    zhash_put (gopt->sopts, &goo->sname, &goo, NULL, NULL);
    zarray_add (gopt->options, &goo);
}

void
getopt_add_int (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help)
{
    getopt_add_string (gopt, sopt, lname, def, help);
}

void
getopt_add_double (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help)
{
    getopt_add_string (gopt, sopt, lname, def, help);
}

void
getopt_add_string (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help)
{
    char sname[2];
    sname[0] = sopt;
    sname[1] = 0;
    char *sname_ptr = (char*) &sname;

    if (strlen (lname) < 1) { // must have long name
        fprintf (stderr, "getopt_add_string(): must supply option name\n");
        exit (EXIT_FAILURE);
    }

    if (sopt == '-') { // short name cannot be '-' (no way to reference)
        fprintf (stderr, "getopt_add_string(): invalid option character: '%c'\n", sopt);
        exit (EXIT_FAILURE);
    }

    if (zhash_contains (gopt->lopts, &lname)) {
        fprintf (stderr, "getopt_add_string(): duplicate option name: --%s\n", lname);
        exit (EXIT_FAILURE);
    }

    if (sopt != '\0' && zhash_contains (gopt->sopts, &sname_ptr)) {
        fprintf (stderr, "getopt_add_string(): duplicate option: -%s ('%s')\n", sname, lname);
        exit (EXIT_FAILURE);
    }

    getopt_option_t *goo = calloc (1, sizeof(*goo));
    goo->sname = strdup (sname);
    goo->lname = strdup (lname);
    goo->svalue = strdup(def);
    goo->type = GOO_STRING_TYPE;
    goo->help = strdup (help);

    zhash_put (gopt->lopts, &goo->lname, &goo, NULL, NULL);
    zhash_put (gopt->sopts, &goo->sname, &goo, NULL, NULL);
    zarray_add (gopt->options, &goo);
}

const char *
getopt_get_string (getopt_t *gopt, const char *lname)
{
    getopt_option_t *goo = NULL;
    zhash_get (gopt->lopts, &lname, &goo);
    // could return null, but this would be the only
    // method that doesn't assert on a missing key
    assert (goo != NULL);
    return goo->svalue;
}

int
getopt_get_int (getopt_t *getopt, const char *lname)
{
    const char *v = getopt_get_string (getopt, lname);
    assert (v != NULL);

    int errno = 0;
    char *endptr = (char *) v;
    int val = strtol (v, &endptr, 10);

    if (errno != 0) {
        fprintf (stderr, "--%s argument: strtol failed: %s\n", lname, strerror(errno));
        exit (EXIT_FAILURE);
    }

    if (endptr == v) {
        fprintf (stderr, "--%s argument cannot be parsed as an int\n", lname);
        exit (EXIT_FAILURE);
    }

    return val;
}

int
getopt_get_bool (getopt_t *getopt, const char *lname)
{
    const char *v = getopt_get_string (getopt, lname);
    assert (v!=NULL);
    int val = 0==strcmp (v, "true");
    return val;
}

double
getopt_get_double (getopt_t *getopt, const char *lname)
{
    const char *v = getopt_get_string (getopt, lname);
    assert (v!=NULL);

    int errno = 0;
    char *endptr = (char *) v;
    double d = strtod (v, &endptr);

    if (errno != 0) {
        fprintf (stderr, "--%s argument: strtod failed: %s\n", lname, strerror(errno));
        exit (EXIT_FAILURE);
    }

    if (endptr == v) {
        fprintf (stderr, "--%s argument cannot be parsed as a double\n", lname);
        exit (EXIT_FAILURE);
    }

    return d;
}

int
getopt_was_specified (getopt_t *getopt, const char *lname)
{
    getopt_option_t *goo = NULL;
    zhash_get (getopt->lopts, &lname, &goo);
    if (goo == NULL)
        return 0;

    return goo->was_specified;
}

const zarray_t *
getopt_get_extra_args (getopt_t *gopt)
{
    return gopt->extraargs;
}

static int
max (int a, int b)
{
    return a > b ? a : b;
}

void
getopt_do_usage (getopt_t *gopt)
{
    int leftmargin=2;
    int longwidth=12;
    int valuewidth=10;

    for (unsigned int i = 0; i < zarray_size (gopt->options); i++) {
        getopt_option_t *goo = NULL;
        zarray_get (gopt->options, i, &goo);

        if (goo->spacer)
            continue;

        longwidth = max (longwidth, strlen(goo->lname));

        if (goo->type == GOO_STRING_TYPE)
            valuewidth = max (valuewidth, strlen(goo->svalue));
    }

    for (unsigned int i = 0; i < zarray_size (gopt->options); i++) {
        getopt_option_t *goo = NULL;
        zarray_get (gopt->options, i, &goo);

        if (goo->spacer) {
            if (goo->help==NULL || 0==strlen (goo->help))
                printf ("\n");
            else
                printf ("\n%*s%s\n\n", leftmargin, "", goo->help);
            continue;
        }

        printf ("%*s", leftmargin, "");

        if (goo->sname[0]==0)
            printf ("     ");
        else
            printf ("-%c | ", goo->sname[0]);

        printf ("--%*s ", -longwidth, goo->lname);

        printf (" [ %s ]", goo->svalue); // XXX: displays current value rather than default value

        printf ("%*s", (int) (valuewidth-strlen (goo->svalue)), "");

        printf (" %s   ", goo->help);
        printf ("\n");
    }
}
