"""
Wheelchair chair-logic software updater.

Responsibilities:
- read the currently installed chair-logic version
- check the version available on GitHub
- download chair_logic.py from GitHub
- validate downloaded Python code
- safely install a new chair_logic.py
- keep one previous version as a rollback backup
- restore the previous version when requested

Only these files are remotely updated:
    /chair_logic.py
    /version.txt

Generated local files:
    /chair_logic_backup.py
    /version_backup.txt
    /chair_logic_new.py
    /version_new.txt

This module does NOT:
- control Wi-Fi
- control MQTT
- interpret remote commands
- control the wheelchair
- reboot the Pico

When imported:
    Nothing happens automatically.

When run directly:
    A safe, non-destructive updater self-test is performed.
"""

import os
import gc

try:
    import requests
except ImportError:
    import urequests as requests


# ============================================================
# GITHUB SETTINGS
# ============================================================

GITHUB_USER = "fields007"
GITHUB_REPO = "wheelchair-pico"
GITHUB_BRANCH = "main"

BASE_URL = (
    "https://raw.githubusercontent.com/"
    + GITHUB_USER
    + "/"
    + GITHUB_REPO
    + "/"
    + GITHUB_BRANCH
    + "/"
)

REMOTE_CHAIR_URL = BASE_URL + "chair_logic.py"
REMOTE_VERSION_URL = BASE_URL + "version.txt"


# ============================================================
# LOCAL FILES
# ============================================================

CHAIR_FILE = "chair_logic.py"
VERSION_FILE = "version.txt"

CHAIR_BACKUP_FILE = "chair_logic_backup.py"
VERSION_BACKUP_FILE = "version_backup.txt"

CHAIR_NEW_FILE = "chair_logic_new.py"
VERSION_NEW_FILE = "version_new.txt"


# ============================================================
# BASIC FILE HELPERS
# ============================================================

def _file_exists(filename):
    """
    Return True if a file exists.
    """

    try:
        os.stat(filename)
        return True

    except OSError:
        return False


def _remove_if_exists(filename):
    """
    Remove a file if it exists.
    """

    try:
        os.remove(filename)

    except OSError:
        pass


def _read_text_file(filename):
    """
    Read an entire text file.

    Raises an exception if reading fails.
    """

    with open(filename, "r") as file:
        return file.read()


def _write_text_file(filename, text):
    """
    Write an entire text file.

    Raises an exception if writing fails.
    """

    with open(filename, "w") as file:
        file.write(text)


def _copy_file(source, destination):
    """
    Copy one local file to another.
    """

    with open(source, "rb") as src:

        with open(destination, "wb") as dst:

            while True:

                block = src.read(512)

                if not block:
                    break

                dst.write(block)


# ============================================================
# VERSION HANDLING
# ============================================================

def current_version():
    """
    Return the locally installed version as a string.

    Returns None if version.txt cannot be read.
    """

    try:

        version = _read_text_file(
            VERSION_FILE
        ).strip()

        if not version:
            return None

        return version

    except Exception:
        return None


def _parse_version(version):
    """
    Try to convert a version such as:

        1
        1.2
        1.2.3
        v1.2.3

    into a tuple of integers.

    Returns None if the version cannot be interpreted numerically.
    """

    if version is None:
        return None

    version = version.strip()

    if version.startswith("v"):
        version = version[1:]

    if not version:
        return None

    try:

        return tuple(
            int(part)
            for part in version.split(".")
        )

    except Exception:

        return None


def compare_versions(local_version, remote_version):
    """
    Compare two version strings.

    Returns:
         1 if remote is newer
         0 if versions are equal
        -1 if remote is older

    If either version is not numeric, exact string comparison
    is used. Different non-numeric versions are treated as
    different, with the remote version considered available.
    """

    local_parsed = _parse_version(
        local_version
    )

    remote_parsed = _parse_version(
        remote_version
    )

    if (
        local_parsed is not None
        and remote_parsed is not None
    ):

        # Make 1.2 equivalent to 1.2.0.
        length = max(
            len(local_parsed),
            len(remote_parsed)
        )

        local_parsed = local_parsed + (
            0,
        ) * (
            length - len(local_parsed)
        )

        remote_parsed = remote_parsed + (
            0,
        ) * (
            length - len(remote_parsed)
        )

        if remote_parsed > local_parsed:
            return 1

        if remote_parsed < local_parsed:
            return -1

        return 0

    if local_version == remote_version:
        return 0

    return 1


# ============================================================
# HTTP
# ============================================================

def _download_text(url):
    """
    Download a text file and return its contents.

    Raises an exception if the request fails.
    """

    response = None

    try:

        gc.collect()

        response = requests.get(
            url
        )

        status = getattr(
            response,
            "status_code",
            200
        )

        if status != 200:

            raise RuntimeError(
                "HTTP status {}".format(
                    status
                )
            )

        text = response.text

        if text is None:

            raise RuntimeError(
                "Downloaded file contains no text."
            )

        return text

    finally:

        if response is not None:

            try:
                response.close()

            except Exception:
                pass

        gc.collect()


def available_version():
    """
    Download and return the version currently available on GitHub.

    Raises an exception if GitHub cannot be reached.
    """

    version = _download_text(
        REMOTE_VERSION_URL
    ).strip()

    if not version:

        raise RuntimeError(
            "Remote version.txt is empty."
        )

    return version


# ============================================================
# UPDATE CHECK
# ============================================================

def check_update():
    """
    Check whether GitHub contains a newer chair-logic version.

    Returns a dictionary:

        {
            "local": "1.0",
            "remote": "1.1",
            "update_available": True
        }

    Raises an exception if the remote version cannot be checked.
    """

    local = current_version()

    if local is None:

        raise RuntimeError(
            "Could not read local version.txt."
        )

    remote = available_version()

    comparison = compare_versions(
        local,
        remote
    )

    return {
        "local": local,
        "remote": remote,
        "update_available": comparison > 0
    }


# ============================================================
# PYTHON VALIDATION
# ============================================================

def validate_python(source):
    """
    Syntax-check Python source without executing it.

    Returns:
        (True, None)

    on success, or:

        (False, error)

    on failure.
    """

    try:

        compile(
            source,
            CHAIR_FILE,
            "exec"
        )

        return True, None

    except Exception as error:

        return False, error


# ============================================================
# DOWNLOAD CANDIDATE
# ============================================================

def download_candidate(log=print):
    """
    Download the remote chair logic and version into temporary
    local files.

    The Python source is syntax-checked before success is
    reported.

    Existing chair_logic.py and version.txt are NOT modified.

    Returns the remote version string.

    Raises an exception on failure.
    """

    _remove_if_exists(
        CHAIR_NEW_FILE
    )

    _remove_if_exists(
        VERSION_NEW_FILE
    )

    try:

        log(
            "Downloading remote version."
        )

        remote_version = available_version()

        log(
            "Remote version: {}".format(
                remote_version
            )
        )

        log(
            "Downloading chair_logic.py."
        )

        source = _download_text(
            REMOTE_CHAIR_URL
        )

        if not source.strip():

            raise RuntimeError(
                "Downloaded chair_logic.py is empty."
            )

        log(
            "Checking downloaded Python syntax."
        )

        valid, error = validate_python(
            source
        )

        if not valid:

            raise RuntimeError(
                "Downloaded chair_logic.py failed syntax check: {}".format(
                    error
                )
            )

        log(
            "Downloaded chair_logic.py passed syntax check."
        )

        _write_text_file(
            CHAIR_NEW_FILE,
            source
        )

        _write_text_file(
            VERSION_NEW_FILE,
            remote_version + "\n"
        )

        return remote_version

    except Exception:

        _remove_if_exists(
            CHAIR_NEW_FILE
        )

        _remove_if_exists(
            VERSION_NEW_FILE
        )

        raise


# ============================================================
# BACKUP
# ============================================================

def create_backup(log=print):
    """
    Create a backup of the currently installed chair logic
    and version.

    Existing backup files are replaced.

    Raises an exception on failure.
    """

    if not _file_exists(
        CHAIR_FILE
    ):

        raise RuntimeError(
            "Current chair_logic.py does not exist."
        )

    if not _file_exists(
        VERSION_FILE
    ):

        raise RuntimeError(
            "Current version.txt does not exist."
        )

    log(
        "Creating chair logic backup."
    )

    _copy_file(
        CHAIR_FILE,
        CHAIR_BACKUP_FILE
    )

    _copy_file(
        VERSION_FILE,
        VERSION_BACKUP_FILE
    )

    log(
        "Backup created."
    )


def backup_available():
    """
    Return True only when both rollback files exist.
    """

    return (
        _file_exists(
            CHAIR_BACKUP_FILE
        )
        and
        _file_exists(
            VERSION_BACKUP_FILE
        )
    )


# ============================================================
# INSTALL UPDATE
# ============================================================

def update(log=print):
    """
    Download and install the current GitHub chair logic.

    Process:
    1. Read current version.
    2. Check remote version.
    3. Refuse downgrade/equal version.
    4. Download candidate.
    5. Syntax-check candidate.
    6. Back up current chair logic and version.
    7. Replace active files.
    8. Remove temporary files.

    This function DOES NOT reboot the Pico.

    Returns True if an update was installed.
    Returns False if no newer version exists.

    Raises an exception if installation fails.
    """

    local_version = current_version()

    if local_version is None:

        raise RuntimeError(
            "Could not read local version.txt."
        )

    log(
        "Installed version: {}".format(
            local_version
        )
    )

    remote_version = available_version()

    log(
        "GitHub version: {}".format(
            remote_version
        )
    )

    comparison = compare_versions(
        local_version,
        remote_version
    )

    if comparison == 0:

        log(
            "Already running the current version."
        )

        return False

    if comparison < 0:

        log(
            "GitHub version is older than installed version."
        )

        return False

    # Download and validate BEFORE touching the working files.
    downloaded_version = download_candidate(
        log=log
    )

    if downloaded_version != remote_version:

        _remove_if_exists(
            CHAIR_NEW_FILE
        )

        _remove_if_exists(
            VERSION_NEW_FILE
        )

        raise RuntimeError(
            "Remote version changed during update."
        )

    # Only now do we touch the existing installation.
    create_backup(
        log=log
    )

    try:

        log(
            "Installing new chair logic."
        )

        # Copy instead of rename so that the temporary files
        # remain available until both active files have been
        # successfully written.
        _copy_file(
            CHAIR_NEW_FILE,
            CHAIR_FILE
        )

        _copy_file(
            VERSION_NEW_FILE,
            VERSION_FILE
        )

        # Verify the installed version file.
        installed_version = current_version()

        if installed_version != remote_version:

            raise RuntimeError(
                "Installed version verification failed."
            )

        log(
            "Update installed successfully."
        )

        log(
            "Installed version: {}".format(
                installed_version
            )
        )

    except Exception as install_error:

        log(
            "Installation failed."
        )

        log(
            "Restoring previous version."
        )

        try:

            _copy_file(
                CHAIR_BACKUP_FILE,
                CHAIR_FILE
            )

            _copy_file(
                VERSION_BACKUP_FILE,
                VERSION_FILE
            )

            log(
                "Previous version restored."
            )

        except Exception as restore_error:

            raise RuntimeError(
                "Update failed: {} ; automatic restore also failed: {}".format(
                    install_error,
                    restore_error
                )
            )

        raise install_error

    finally:

        _remove_if_exists(
            CHAIR_NEW_FILE
        )

        _remove_if_exists(
            VERSION_NEW_FILE
        )

    return True


# ============================================================
# ROLLBACK
# ============================================================

def rollback(log=print):
    """
    Restore the previous chair logic and version.

    The currently installed version is NOT made into the new
    backup. The backup remains the known previous version.

    This function DOES NOT reboot the Pico.

    Returns True on success.

    Raises an exception if no complete backup exists or the
    restore fails.
    """

    if not backup_available():

        raise RuntimeError(
            "No complete rollback backup is available."
        )

    log(
        "Restoring chair logic backup."
    )

    # Validate backup code before replacing the active file.
    backup_source = _read_text_file(
        CHAIR_BACKUP_FILE
    )

    valid, error = validate_python(
        backup_source
    )

    if not valid:

        raise RuntimeError(
            "Backup chair logic failed syntax check: {}".format(
                error
            )
        )

    backup_version = _read_text_file(
        VERSION_BACKUP_FILE
    ).strip()

    if not backup_version:

        raise RuntimeError(
            "Backup version is empty."
        )

    _copy_file(
        CHAIR_BACKUP_FILE,
        CHAIR_FILE
    )

    _copy_file(
        VERSION_BACKUP_FILE,
        VERSION_FILE
    )

    log(
        "Rollback completed."
    )

    log(
        "Restored version: {}".format(
            backup_version
        )
    )

    return True


# ============================================================
# SELF-TEST
# ============================================================

def self_test():
    """
    Perform a safe updater self-test.

    This test:
    - connects to Wi-Fi if necessary
    - reads the installed version
    - reads the GitHub version
    - compares the versions
    - downloads GitHub chair_logic.py
    - syntax-checks the downloaded code
    - removes temporary files afterwards

    It DOES NOT:
    - modify chair_logic.py
    - modify version.txt
    - modify backups
    - install an update
    - perform a rollback
    - reboot the Pico
    """

    print()
    print("==============================")
    print("UPDATER SELF-TEST")
    print("==============================")
    print()

    # --------------------------------------------------------
    # LOCAL VERSION
    # --------------------------------------------------------

    print("1. Reading installed version...")

    local_version = current_version()

    if local_version is None:

        print(
            "FAILED: Could not read version.txt."
        )

        return

    print(
        "OK: Installed version: {}".format(
            local_version
        )
    )

    # --------------------------------------------------------
    # WIFI
    # --------------------------------------------------------

    print()
    print("2. Checking Wi-Fi...")

    try:

        from system import wifi_manager

    except ImportError:

        # Useful if this file is run directly in a different
        # MicroPython import context.
        import wifi_manager

    if not wifi_manager.is_connected():

        print(
            "Wi-Fi is not connected. Connecting..."
        )

        if not wifi_manager.connect():

            print()
            print(
                "FAILED: Could not connect to Wi-Fi."
            )

            return

    print(
        "OK: Wi-Fi connected."
    )

    if wifi_manager.current_ssid() is not None:

        print(
            "SSID: {}".format(
                wifi_manager.current_ssid()
            )
        )

    if wifi_manager.ip_address() is not None:

        print(
            "IP: {}".format(
                wifi_manager.ip_address()
            )
        )

    # --------------------------------------------------------
    # REMOTE VERSION
    # --------------------------------------------------------

    print()
    print("3. Reading GitHub version...")

    try:

        remote_version = available_version()

    except Exception as error:

        print(
            "FAILED: {}".format(
                error
            )
        )

        return

    print(
        "OK: GitHub version: {}".format(
            remote_version
        )
    )

    # --------------------------------------------------------
    # VERSION COMPARISON
    # --------------------------------------------------------

    print()
    print("4. Comparing versions...")

    comparison = compare_versions(
        local_version,
        remote_version
    )

    if comparison > 0:

        print(
            "A newer version is available."
        )

    elif comparison < 0:

        print(
            "The installed version is newer than GitHub."
        )

    else:

        print(
            "Installed and GitHub versions are the same."
        )

    # --------------------------------------------------------
    # DOWNLOAD + VALIDATION
    # --------------------------------------------------------

    print()
    print(
        "5. Downloading and validating GitHub chair_logic.py..."
    )

    # Important: ensure the test starts without stale temporary
    # files.
    _remove_if_exists(
        CHAIR_NEW_FILE
    )

    _remove_if_exists(
        VERSION_NEW_FILE
    )

    try:

        downloaded_version = download_candidate(
            log=print
        )

        print()
        print(
            "OK: Candidate version {} downloaded and validated.".format(
                downloaded_version
            )
        )

    except Exception as error:

        print()
        print(
            "FAILED: {}".format(
                error
            )
        )

        return

    finally:

        # The self-test must leave no candidate files behind.
        _remove_if_exists(
            CHAIR_NEW_FILE
        )

        _remove_if_exists(
            VERSION_NEW_FILE
        )

    # --------------------------------------------------------
    # VERIFY NON-DESTRUCTIVE CLEANUP
    # --------------------------------------------------------

    print()
    print("6. Checking self-test cleanup...")

    if (
        _file_exists(CHAIR_NEW_FILE)
        or
        _file_exists(VERSION_NEW_FILE)
    ):

        print(
            "FAILED: Temporary files remain."
        )

        return

    if current_version() != local_version:

        print(
            "FAILED: Installed version changed during test."
        )

        return

    print(
        "OK: Installed files were not changed."
    )

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    print()
    print("==============================")
    print("UPDATER SELF-TEST PASSED")
    print("==============================")
    print()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    self_test()