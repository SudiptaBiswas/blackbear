#!/usr/bin/env bash
###
###
###     $$\   $$\ $$\  $$$$$$\  $$\      $$\ $$\      $$\  $$$$$$\
###     $$$\  $$ |\__|$$  __$$\ $$$\    $$$ |$$$\    $$$ |$$  __$$\
###     $$$$\ $$ |$$\ $$ /  \__|$$$$\  $$$$ |$$$$\  $$$$ |$$ /  $$ |
###     $$ $$\$$ |$$ |$$ |      $$\$$\$$ $$ |$$\$$\$$ $$ |$$$$$$$$ |
###     $$ \$$$$ |$$ |$$ |      $$ \$$$  $$ |$$ \$$$  $$ |$$  __$$ |
###     $$ |\$$$ |$$ |$$ |  $$\ $$ |\$  /$$ |$$ |\$  /$$ |$$ |  $$ |
###     $$ | \$$ |$$ |\$$$$$$  |$$ | \_/ $$ |$$ | \_/ $$ |$$ |  $$ |
###     \__|  \__|\__| \______/ \__|     \__|\__|     \__|\__|  \__|
###
### NiCMMA - Nightly Computational Material & Mechanics Assessment script
###
### Script to launch nightly pbs testing for MOOSE herd applications.
###
### Usage:
###    nicmma.sh "bison" "assessment examples" "john.doe@gmail.com alice@inl.gov"
###
### Depends on:
###    modules - use.moose PETSc
###    git
###    qsub
###    mail
###
### (c) 2020 Battelle Energy Alliance, LLC - ALL RIGHTS RESERVED

#######################################################################
################################ Notes ################################
#######################################################################
## Semantics:
##
##  - Default variables, including variables specified by the input
##    parameters are prepended with an underscore.
##    (e.g. "_JOBS","_CODE_NAME").
##
##  - Global variables that are dependent on default variables or not
##    constant are prepended with a double-underscore
##    (e.g. "__FULL_PATH", "__OUTPUT_FILE").
##
##  - Variables local to a function are written in lowercase
##    (e.g. "test_type", "input_file").
##
##  - Variables defined in the shell environment are written in all-caps,
##    with no leading underscores (e.g. "MOOSE_JOBS", "PIPESTATUS").
##
##  - Always use braces when referencing variables. Prefer "${NAME}"
##    over "$NAME".
##
##  - All HEREDOCS with a leading '<<-' contain lines with a leading
##    tab-character. The rest of the file contains only spaces.

#######################################################################
############################## Environment ############################
#######################################################################
readonly _ME="nicmma.sh"
readonly _OWNER_EMAIL="sudipta.biswas@inl.gov"
readonly _JOBS=48
readonly _DATE_CODE="$(date "+%Y%m%d")"
readonly _BINARIES_IN_USE=(qsub git mail)
readonly _HOMEBASE="~/Programs/blackbear/cmm-nightly"

## Set $IFS to only newline and tab.
##
## http://www.dwheeler.com/essays/filenames-in-shell.html
IFS=$'\n\t'

# default params
_USE_DEBUG=0
_HEAVY_RUN=0
_PLOT_DIFF=0

# global mutable variable
__DEBUG_COUNTER=0

# Predefine these input variables
_CODE_NAME= blackbear
_EMAIL_USERS=(sudipta.biswas@inl.gov)
_TEST_CASES=(wald2017B)

#######################################################################
############################## Utilities ##############################
#######################################################################
## usage()
##
## Print the program help information.
usage() {
  ## Note: all these lines are prepended with actual tab characters to
  ## preserve code formating. They are removed by the heredoc.
  cat <<-EOF
	USAGE:
	    ${_ME} [-h] [-d] [-e] CODENAME TESTS EMAILS

	ARGUMENTS:
	    CODENAME    Application animal name
	    TESTS       Space delimited list of tests to run
	    EMAILS      Space delimited list of emails

	OPTIONS:
	    -h     Show this message.
	    -d     Enable debugging features.
	    -e     Enable heavy testing.
	    -p     Enable plot-diffing email.

	EXAMPLES:
	    ${_ME} "bison" "examples assessment" "john@doe.net alice@gmail.com"
	    ${_ME} -e -d "grizzly" "examples" "grizzly-nightly@inl.gov"
	EOF
}

## debug()
##
## A simple function for executing a specified command if the
## `_USE_DEBUG' variable has been set.
# debug() {
#   if [[ "${_USE_DEBUG:-"0"}" == 1 ]]; then
#     __DEBUG_COUNTER=$((__DEBUG_COUNTER + 1))
#     # Prefix debug message with "bug (U+1F41B)"
#     printf "DEBUG!  %s " "${__DEBUG_COUNTER}"
#     echo "${@}"
#     printf "%s\n" "-------------------------------------------------"
#   fi
# }

## die()
##
## A simple function for exiting with an error after executing the
## specified command.
# die() {
#   # Prefix die message with "cross mark (U+274C)", displayed as a red x.
#   printf "❌  "
#   echo "${@}" 1>&2
#   exit 1
# }

#######################################################################
################################# Core ################################
#######################################################################
## git_latest_commit()
##
## A simple function to fetch the latest commit from a repository.
## You do not need to be in a git repository to run this command.
# git_latest_commit() {
#   git ls-remote "$1" HEAD | cut -f1 | uniq
# }

## ensure_prereqs()
##
## This function checks for dependencies and folder existence.
# ensure_prereqs() {
#   debug ">> Ensuring all prerequisites..."
#   [[ -d "${__PROJECT_PATH}" ]] ||
#     die "Directory: ${__PROJECT_PATH} not found"
#
#   for binary in "${_BINARIES_IN_USE[@]}"; do
#     [[ $(command -v "${binary}") ]] || die "${binary} not found!"
#   done
# }

## git_clone_project()
##
## This function creates a local git repository and fetches the latest
## commit from the _CODE_NAME repo. If it can't fetch the latest
## commit it will kill the script.
# git_clone_project() {
#   debug ">> Cloning ${_CODE_NAME} into ${__FULL_PATH}..."
#   git init "${__FULL_PATH}"
#   cd "${__FULL_PATH}" || die "Error ${__FULL_PATH} does not exist!"
#
#   git config --local gc.auto 0
#   git config --local protocol.version 2
#   git remote add origin "${__GIT_REPO}"
#
#   # Enforce successful completion of git fetch.
#   if ! {
#       git fetch --no-tags --prune --progress \
#           --no-recurse-submodules --depth=1 origin devel
#       git checkout --progress \
#           --force -B devel refs/remotes/origin/devel
#       git submodule update --init
#   }; then
#     die "Cloning remote repo failed!"
#   fi
#
# }

## build_libMesh()
##
## This function ensures we are in the proper repository and then
## attempts to build libMesh.
# build_libMesh() {
#   debug ">> Building libMesh..."
#   export MOOSE_JOBS="${_JOBS}"
#   eval "${__FULL_PATH}/moose/scripts/update_and_rebuild_libmesh.sh"
# }

## build_app()
##
## This function runs `make` inside the _CODE_NAME repo.
# build_app() {
#   debug ">> Building ${_CODE_NAME} App..."
#   make -j "${_JOBS}"
# }

## launch_test()
##
## This is the function that launches the first set of tests.
## Do not background initial test run.
# launch_test() {
#   local input_file="${1}"
#   local pbs_file="${_DATE_CODE}_PBS_${input_file}"
#   local prof_file="${_HOMEBASE}/.profile"
#
#   debug ">> Launching ${input_file} tests..."
#
#   if [[ "${_HEAVY_RUN:-"0"}" == 1 ]]; then
#     ./run_tests \
#       --all-tests \
#       --no-color \
#       -t \
#       -x \
#       --pbs-pre-source "${prof_file}" \
#       --pbs "${pbs_file}" \
#       -i "${input_file}" 2>&1
#   else
#     ./run_tests \
#       --no-color \
#       -t \
#       -x \
#       --pbs-pre-source "${prof_file}" \
#       --pbs "${pbs_file}" \
#       -i "${input_file}" 2>&1
#   fi
# }

## change_permissions()
##
## This function runs after the tests complete to change the
## permissions in the _CODE_NAME directory.
# change_permissions() {
#   debug ">> Changing permissions..."
#   local test_type
#   for test_type in "${_TEST_CASES[@]}"; do
#     chgrp -R "${_CODE_NAME}" "${__FULL_PATH}/${test_type}" 2>/dev/null
#     chmod -R g+rw "${__FULL_PATH}/${test_type}" 2>/dev/null
#   done
# }

## are_queued_jobs_finished()
##
## This function runs an inline python script to check pbs for finished
## jobs. This previously was a script in moose/scripts, but in order to
## reduce dependency on external processeses, it was inlined here.
are_queued_jobs_finished() {
  debug ">> Checking if queued jobs are finished..."
  local results_file="${1}"
  # The python script requires a results file to be passed as a param.
  python "${_HOMEBASE}/are_queued_jobs_finished.py" "${results_file}"
}

## record_results()
##
## This function dumps the output information to a text file in the
## _CODE_NAME folder.
record_results() {
  local test_type="${1}"
  local output_file="${2}"
  local pbs_file="${_DATE_CODE}_PBS_${test_type}"
  local prof_file="${_HOMEBASE}/.profile"

  debug ">> Recording ${test_type} results..."

  if [[ "${_HEAVY_RUN:-"0"}" == 1 ]]; then
    ./run_tests \
      --all-tests \
      --no-color \
      -t \
      --pbs-pre-source "${prof_file}" \
      --max-fails 100 \
      --pbs "${pbs_file}" \
      -i "${test_type}" > "${output_file}"
  else
    ./run_tests \
      --no-color \
      -t \
      --pbs-pre-source "${prof_file}" \
      --max-fails 100 \
      --pbs "${pbs_file}" \
      -i "${test_type}" > "${output_file}"
  fi
}

## is_testing_complete()
##
## This function gets the information from the current output file to
## determine if testing is complete. It will return either a 0 if
## successful or a 1 if failure.
##
## Currently, the function is 'unused' as it has no effects during
## runtime.
is_testing_complete() {
  local output_file="${1}"
  # check last lines of output file to see if ' 0 pending' is there.
  [[ $(tail -n3 "${output_file}" | grep -c "[[:space:]][0] pending") -eq 1 ]] &&
    return 0
  return 1
}

## is_test_passed()
##
## This function returns an integer value. 0 for passsed, 1 for did not
## pass.
# is_test_passed() {
#   debug ">> Checking if tests passed..."
#   local output_file="${1}"
#   # check last line of output file to see if ' 0 failed' is there.
#   [[ $(tail -n3 "${output_file}" | grep -c -i '[[:space:]][0] failed') -eq 1 ]] &&
#     return 0
#   return 1
# }

## construct_message()
##
## This function creates a message that will be sent in the body of the
## email. The default message is written in HTML.
construct_message() {
  local test_type="${1}"
  local output_file="${2}"
  local message

  debug ">> Contructing message for ${test_type}..."

  ## Note: all lines in the EOM heredoc are prepended
  ## with actual tab characters to preseve formating.
  read -r -d '' message <<-EOM
	<h3>[./run_tests ${__PROJECT} ${test_type}] results:</h3>
	<p style='font-weight: bold;'>Commit Hash: ${__LATEST_COMMIT}</p>
	<pre style='font-size:12px;'>
	Loaded Modules:
	\t${LOADEDMODULES//:/'\n\t'}

	PETSc Directory:
	\t${PETSC_DIR}
	EOM

  # Concat actual test results to end of boilerplat message.
  if [[ "$(grep -c "Final Test Results" "${output_file}")" == "0" ]]; then
    __MESSAGE="${message}\n\n$(cat "${output_file}")\n</pre>"
  else
    # Only output file starting from 'Final Test Results'.
    __MESSAGE="${message}\n\n$(sed -n '/Final Test Results/,$p' <"${output_file}")\n</pre>"
  fi
}

## send_email()
##
## This function sends the email out to every user in the emails array.
##
## There are some peculiarities with getting the html messages to send
## correctly. This function creates a header that gets appened to the top
## of the email that specifices MIME types and HTML specs.
send_email() {
  local subject="${1}"
  local message="${2}"
  local tmp_message
  local email
  local is_heavy

  if [[ "${_HEAVY_RUN:-"0"}" == 1 ]]; then
    is_heavy="Heavy"
  else
    is_heavy=""
  fi

  tmp_message="$(mktemp)"
  echo -e <<-EOM > "${tmp_message}"
	To: ${email}
	Subject: *** ${__TEST_FOLDER} ${is_heavy} ${subject} ***
	Content-Type: text/html
	MIME-Version: 1.0

	${message}
	EOM

  for email in "${_EMAIL_USERS[@]}"; do
    debug ">> Sending email to ${email}..."
    /usr/sbin/sendmail -t <"${tmp_message}"
  done
}

## run_the_gauntlet()
##
## This function will control running through all the specified test
## cases. It will then check their exit status and determine which
## email to send out.
run_the_gauntlet() {
  local test_type
  local pretest_file
  local output_file
  debug ">> RUN THE GAUNTLET!..."
  for test_type in "${_TEST_CASES[@]}"; do
    pretest_file="${__FULL_PATH}/${_DATE_CODE}_PBS_${test_type}"
    output_file="${__FULL_PATH}/${_DATE_CODE}_RESULTS_${test_type}"
    launch_test "${test_type}"

    # Every 5 minutes check to see if our queued jobs are finished.
    while :
    do
      sleep 300
      are_queued_jobs_finished "${pretest_file}" && break
    done

    ## If you're having trouble accessing the nightly run output, it's
    ## probably because these permissions haven't changed back yet.
    change_permissions

    record_results "${test_type}" "${output_file}"

    # Non-sequiter function
    is_testing_complete "${output_file}"

    construct_message "${test_type}" "${output_file}"

    if is_test_passed "${output_file}"; then
      send_email "${test_type} SUCCESS" "${__MESSAGE}"
    else
      send_email "${test_type} FAIL" "${__MESSAGE}"
    fi
  done
}

## post_test()
##
## This function runs the post-test commands.
##
## I'm not sure the direct use for this function but it was ported
## directly from the original nightly.sh script.
##
## This function does not run during heavy testing.
post_test() {
  debug ">> Running post tests..."
  ./run_tests -c \
    --pbs-pre-source "${_HOMEBASE}/.profile" \
    -i post > "${__FULL_PATH}/post.log"
}

## plot_diffs()
##
## This function runs a plotting script on all assessment cases that
## failed to due CSVDIFF during the night's run. It stores the .png
## in the same directory as the chkfile.
plot_diffs() {
  debug ">> Plotting all diffed cases..."
  ## We need to find all *DIFF*.txt files to get the name of the case
  ## that diffed. We will then use this array to find all *_chkfile.csvs
  ## that contain the data to plot
  mapfile -t diffed < <(find "${__FULL_PATH}/assessment" \
                             -type f \
                             -name "*DIFF*" \
                             -printf "%h\n")

  # If no cases diffed then stop here.
  [[ ${#diffed[@]} -eq 0 ]] && return 1

  # Out of all cases that diffed, find its' respective chkfile.
  mapfile -t output < <(for i in "${diffed[@]}"; do
                          find "${i}" \
                               -type f \
                               -name "*_chkfile.csv" \
                               -not -path "**/gold/**" \
                               -printf "%p\n"
                        done)
  local chkfile
  for chkfile in "${output[@]}"; do
    ## It's assumed that the gold file has the exact same name as the
    ## chkfile just in the /gold/ folder.
    gold_file="$(dirname "${chkfile}")/gold/$(basename "${chkfile}")"
    ## Python script takes two parameters.
    ## The chkfile (-o) and the gold file (-g).
    python "${_HOMEBASE}/diff_overlay.py" -o "${chkfile}" -g "${gold_file}"
  done
}

## construct_diff_message()
##
## This function creates an email to send out with all plot diffs using
## a base64 encoding of the image.
# construct_diff_message() {
#   debug ">> Emailing diffed plots..."
#   # Make sure to exclude any images found in /docs/ or /gold/.
#   local imgs
#   mapfile -t imgs < <(find "${__FULL_PATH}/assessment" \
#                            -type f \
#                            -name "*_chkfile.png" \
#                            -not -path "**/doc/**" \
#                            -not -path "**/gold/**" \
#                            -printf "%p\n")
#
#   local img
#   for img in "${imgs[@]}"; do
#     /usr/bin/base64 "${img}" > "${img}.base64"
#   done
#   unset img
#
#   # Now collect all the codified image paths into an array.
#   local codimg
#   mapfile -t codimg < <(find "${__FULL_PATH}/assessment" \
#                              -type f \
#                              -name "*.base64" \
#                              -printf "%p\n")
#
#   local tmp_message
#   local message
#   read -r -d '' message <<-EOT
# 	SUBJECT: BISON -- Plots of Diffing Cases
# 	MIME-Version: 1.0
# 	Content-Type: multipart/related;boundary="XYZ"
#
# 	--XYZ
# 	Content-Type: text/html; charset=ISO-8859-15
# 	Content-Transfer-Encoding: 7bit
#
# 	<html>
# 	<head>
# 	<meta http-equiv="content-type" content="text/html; charset=ISO-8859-15">
# 	</head>
# 	<body bgcolor="#ffffff" text="#000000">
# 	<h1>Plots of Diffing Assessment Cases.</h1>
# 	$(for img in "${imgs[@]}"; do
# 	    echo "<h2>$(basename $img)</h2>"
# 	    echo "<img src=\"cid:${img}\"></br>";
# 	done)
# 	</body>
# 	</html>
#
# 	$(for ((i = 0; i < ${#imgs[@]}; i++)); do
# 	    echo "--XYZ"
# 	    echo "Content-Type: image/png;name=\"${imgs[i]}\""
# 	    echo "Content-Transfer-Encoding: base64"
# 	    echo "Content-ID: <${imgs[i]}>"
# 	    echo "Content-Disposition: inline; filename=\"${imgs[i]}\""
# 	    echo ""
# 	    echo "$(cat ${codimg[i]})"
# 	done)
# 	--XYZ--
# 	EOT
#
#   for email in "${_EMAIL_USERS[@]}"; do
#     tmp_message=$(mktemp)
#     echo -e "TO: ${email}\n${message}" >"${tmp_message}"
#     /usr/sbin/sendmail -t <"${tmp_message=}"
#   done
#
#   # Remove all base64 images, but keep the .pngs for later inspection.
#   local cod
#   for cod in "${codimg[@]}"; do
#     rm "${cod}"
#   done
# }
#
# ## cleanup()
# ##
# ## This function moves the log file to the proper location and exits
# ## the script.
# cleanup() {
#   debug ">> Cleaning up our mess..."
#
#   # There might be a better place to run the plot-differ email.
#   if [[ "${_PLOT_DIFF:-"0"}" == "1" ]]; then
#       plot_diffs && construct_diff_message
#   fi
#
#   ## When redirecting git output, it gets all jumbled up.
#   ## Remove '\r' in favor of '\n' for the log file.
#   sed -i 's/\r/\n/g' "${__LOG_FILE}"
#   mv "${__LOG_FILE}" "${__FULL_PATH}/"
#   exit 0
# }

#######################################################################
############################### Options ###############################
#######################################################################
## parse_options()
##
## This function parses the given options and makes it easier to
## add options in the future.
parse_options() {
  while getopts ":hdep" flag; do
    case "$flag" in
      h)
        usage
        exit 0
        ;;
      d)
        _USE_DEBUG=1
        ;;
      e)
        _HEAVY_RUN=1
        ;;
      p)
        _PLOT_DIFF=1
        ;;
      \?)
        usage
        exit 1
        ;;
    esac
  done

  shift $((OPTIND - 1))
  # Handle non-option arguments.
  if [[ ${#} -ne 3 ]]; then
    echo "$0: Three input parameters are required"
    exit 4
  fi

  _CODE_NAME="${1}"  # e.g (bison|grizzly)
  # Proper way to read space-delimted strings into an array.
  OIFS="${IFS}"
  IFS=' '
  read -r -a _TEST_CASES <<<"${2}"
  read -r -a _EMAIL_USERS <<<"${3}"
  IFS="${OIFS}"

  ## These variables depend on either input parameters
  ## or are expected to change.
  __PROJECT_PATH="~/Programs/${_CODE_NAME}"
  __TEST_FOLDER="${_CODE_NAME}_${_DATE_CODE}"
  __FULL_PATH="${__PROJECT_PATH}/${__TEST_FOLDER}"

  __GIT_REPO="git@hpcgitlab.inl.gov:idaholab/${_CODE_NAME}.git"
  __LATEST_COMMIT="$(git_latest_commit "${__GIT_REPO}")"
  __LOG_FILE="${_HOMEBASE}/nicmma_${_CODE_NAME}_${_DATE_CODE}.log"
  __PROJECT="${__TEST_FOLDER}"
}

######################################################################
############################### Logging ##############################
######################################################################
## start_logging()
##
## This function logs everything to a file called
## nicmma_CODENAME_DATE.log. It is initially stored in the cmm-nightly
## folder, but will be moved after everything runs successfully.
start_logging() {
  [[ -e "${__LOG_FILE}" ]] && rm "${__LOG_FILE}"

  exec > >(gawk -v pid=$$ '{ print strftime("%F-%T"),pid,$0; fflush(); }' |& tee -a "${__LOG_FILE}")

  if [[ "${_USE_DEBUG}" -eq 1 ]]; then
    exec 2>&1
  else
    exec 2> >(gawk -v pid=$$ '{ print strftime("%F-%T"),pid,$0; fflush(); }' >>"${__LOG_FILE}")
  fi

  echo "=== Log started for $$ at $(date +%F-%T) ==="
}

## set_script_options()
##
## This function sets all the options for the script. This will enforce
## the script to run in "Strict" mode. This ensures that all variables
## are defined and no unexpected error occur during runtime.
set_script_options() {
  debug ">> Setting up Script Options..."
  set -o nounset    # exit on non-defined variables
  trap 'echo "Aborting due to errexit on line $LINENO. Exit code: $?"' ERR >&2
}

#######################################################################
################################# Main ################################
#######################################################################
main() {
  set_script_options
  parse_options "${@}"
  start_logging
  ensure_prereqs
  git_clone_project
  build_libMesh
  build_app
  run_the_gauntlet
  [[ "${_HEAVY_RUN:-"0"}" == "0" ]] && post_test
  cleanup
}

## If this script gets sourced, it will just load the functions into
## the environment. This is helpful for unit testing individual
## functions.
if [[ "${BASH_SOURCE[0]}" = "${0}" ]]; then
  main "${@}"
fi
