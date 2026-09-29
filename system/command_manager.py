"""
Remote command manager.

Responsibilities:
- receive command strings from MQTT
- parse and validate commands
- queue commands for main.py to process
- provide access to the pending command

This module does NOT:
- use MQTT directly
- connect to Wi-Fi
- perform software updates
- reboot the Pico
- control the wheelchair

The MQTT callback should call:

    command_manager.receive(command)

main.py can then inspect:

    command_manager.pending_command()

and remove it using:

    command_manager.take_command()

Supported commands:

    status
    check-update
    update
    rollback
    reboot
    logs-on
    logs-off
    add-wifi|SSID|PASSWORD

When imported:
    Nothing happens automatically.

When run directly:
    A safe command parser self-test is performed.
"""


# ============================================================
# SUPPORTED COMMANDS
# ============================================================

COMMAND_STATUS = "status"
COMMAND_CHECK_UPDATE = "check-update"
COMMAND_UPDATE = "update"
COMMAND_ROLLBACK = "rollback"
COMMAND_REBOOT = "reboot"
COMMAND_LOGS_ON = "logs-on"
COMMAND_LOGS_OFF = "logs-off"
COMMAND_ADD_WIFI = "add-wifi"


# ============================================================
# STATE
# ============================================================

_pending_command = None


# ============================================================
# PARSING
# ============================================================

def parse(command_text):
    """
    Parse a command string.

    Returns a dictionary on success.

    Examples:

        status

    becomes:

        {
            "command": "status"
        }

    and:

        add-wifi|My WiFi|secret password

    becomes:

        {
            "command": "add-wifi",
            "ssid": "My WiFi",
            "password": "secret password"
        }

    Returns None for an invalid or unsupported command.
    """

    if not isinstance(command_text, str):
        return None

    command_text = command_text.strip()

    if not command_text:
        return None

    # --------------------------------------------------------
    # SIMPLE COMMANDS
    # --------------------------------------------------------

    simple_commands = (
        COMMAND_STATUS,
        COMMAND_CHECK_UPDATE,
        COMMAND_UPDATE,
        COMMAND_ROLLBACK,
        COMMAND_REBOOT,
        COMMAND_LOGS_ON,
        COMMAND_LOGS_OFF,
    )

    if command_text in simple_commands:

        return {
            "command": command_text
        }

    # --------------------------------------------------------
    # ADD WIFI
    # --------------------------------------------------------

    prefix = COMMAND_ADD_WIFI + "|"

    if command_text.startswith(prefix):

        parts = command_text.split(
            "|",
            2
        )

        if len(parts) != 3:
            return None

        ssid = parts[1]
        password = parts[2]

        # Spaces inside the SSID and password are preserved.
        # Only an actually empty SSID is rejected.
        if not ssid:
            return None

        return {
            "command": COMMAND_ADD_WIFI,
            "ssid": ssid,
            "password": password
        }

    # --------------------------------------------------------
    # UNKNOWN COMMAND
    # --------------------------------------------------------

    return None


# ============================================================
# RECEIVE COMMAND
# ============================================================

def receive(command_text, log=print):
    """
    Parse and queue an incoming command.

    Intended to be used as the MQTT command callback.

    This function deliberately does NOT execute the command.

    Returns True if the command was valid and queued.
    Returns False if it was invalid or unsupported.
    """

    global _pending_command

    parsed = parse(
        command_text
    )

    if parsed is None:

        log(
            "Unknown or invalid command: {}".format(
                command_text
            )
        )

        return False

    # There is intentionally only one command slot.
    #
    # Remote commands are maintenance operations, not a
    # high-throughput command stream. Keeping one pending
    # command makes the state simple and predictable.
    #
    # Do not overwrite a command that main.py has not yet
    # processed.
    if _pending_command is not None:

        log(
            "Command ignored because another command is pending."
        )

        return False

    _pending_command = parsed

    log(
        "Command received: {}".format(
            parsed["command"]
        )
    )

    return True


# ============================================================
# PENDING COMMAND ACCESS
# ============================================================

def has_pending_command():
    """
    Return True if a command is waiting to be processed.
    """

    return _pending_command is not None


def pending_command():
    """
    Return the pending command without removing it.

    Returns None if no command is pending.
    """

    return _pending_command


def take_command():
    """
    Return and remove the pending command.

    Returns None if no command is pending.
    """

    global _pending_command

    command = _pending_command

    _pending_command = None

    return command


def clear():
    """
    Remove any pending command.
    """

    global _pending_command

    _pending_command = None


# ============================================================
# COMMAND PROPERTIES
# ============================================================

def requires_chair_stop(command):
    """
    Return whether a parsed command should cause the chair
    control loop to stop before the command is executed.

    update:
        Yes. Files will be replaced.

    rollback:
        Yes. Files will be replaced.

    reboot:
        Yes. The Pico will reset.

    status:
        No.

    check-update:
        No from a logical perspective, although main.py may
        choose when to perform the network request.

    logs-on:
        No. Only changes remote telemetry state.

    logs-off:
        No. Only changes remote telemetry state.

    add-wifi:
        No. It only modifies saved Wi-Fi configuration.
    """

    if command is None:
        return False

    command_name = command.get(
        "command"
    )

    return command_name in (
        COMMAND_UPDATE,
        COMMAND_ROLLBACK,
        COMMAND_REBOOT
    )


# ============================================================
# DESCRIPTION
# ============================================================

def describe(command):
    """
    Produce a human-readable description of a parsed command.

    Passwords are deliberately never included.
    """

    if command is None:
        return "No command"

    command_name = command.get(
        "command"
    )

    if command_name == COMMAND_ADD_WIFI:

        return "Add Wi-Fi network '{}'".format(
            command.get(
                "ssid",
                ""
            )
        )

    return command_name


# ============================================================
# SELF-TEST HELPERS
# ============================================================

def _test_parse(
    text,
    expected_command,
    expected_ssid=None,
    expected_password=None
):
    """
    Test one valid command.

    Returns True on success.
    """

    result = parse(
        text
    )

    if result is None:

        print(
            "FAILED: {!r} was rejected.".format(
                text
            )
        )

        return False

    if result.get("command") != expected_command:

        print(
            "FAILED: {!r} parsed as {!r}.".format(
                text,
                result
            )
        )

        return False

    if expected_ssid is not None:

        if result.get("ssid") != expected_ssid:

            print(
                "FAILED: SSID parsing for {!r}.".format(
                    text
                )
            )

            return False

    if expected_password is not None:

        if result.get("password") != expected_password:

            print(
                "FAILED: Password parsing for {!r}.".format(
                    text
                )
            )

            return False

    print(
        "OK: {!r} -> {}".format(
            text,
            describe(result)
        )
    )

    return True


def _test_invalid(text):
    """
    Test that an invalid command is rejected.

    Returns True on success.
    """

    result = parse(
        text
    )

    if result is not None:

        print(
            "FAILED: {!r} should have been rejected.".format(
                text
            )
        )

        return False

    print(
        "OK: {!r} correctly rejected.".format(
            text
        )
    )

    return True


# ============================================================
# SELF-TEST
# ============================================================

def self_test():
    """
    Perform a completely safe command-manager self-test.

    The test:
    - parses all supported commands
    - tests Wi-Fi credentials containing spaces
    - tests invalid commands
    - tests the one-command queue
    - tests command removal
    - tests stop-required classification

    It DOES NOT:
    - connect to Wi-Fi
    - connect to MQTT
    - alter wifi.json
    - contact GitHub
    - update files
    - reboot the Pico
    - control any hardware
    """

    print()
    print("==============================")
    print("COMMAND MANAGER SELF-TEST")
    print("==============================")
    print()

    passed = True

    # Start with known state.
    clear()

    # --------------------------------------------------------
    # SIMPLE COMMAND PARSING
    # --------------------------------------------------------

    print("1. Testing simple commands...")

    tests = (
        (
            "status",
            COMMAND_STATUS
        ),
        (
            "check-update",
            COMMAND_CHECK_UPDATE
        ),
        (
            "update",
            COMMAND_UPDATE
        ),
        (
            "rollback",
            COMMAND_ROLLBACK
        ),
        (
            "reboot",
            COMMAND_REBOOT
        ),
        (
            "logs-on",
            COMMAND_LOGS_ON
        ),
        (
            "logs-off",
            COMMAND_LOGS_OFF
        ),
    )

    for text, expected in tests:

        if not _test_parse(
            text,
            expected
        ):

            passed = False

    # --------------------------------------------------------
    # ADD WIFI PARSING
    # --------------------------------------------------------

    print()
    print("2. Testing add-wifi...")

    if not _test_parse(
        "add-wifi|Home WiFi|hello123",
        COMMAND_ADD_WIFI,
        expected_ssid="Home WiFi",
        expected_password="hello123"
    ):

        passed = False

    if not _test_parse(
        "add-wifi|Jan's Phone|a password with spaces",
        COMMAND_ADD_WIFI,
        expected_ssid="Jan's Phone",
        expected_password="a password with spaces"
    ):

        passed = False

    # The third field is allowed to contain "|" characters
    # because split("|", 2) only splits twice.
    if not _test_parse(
        "add-wifi|Test Network|abc|123",
        COMMAND_ADD_WIFI,
        expected_ssid="Test Network",
        expected_password="abc|123"
    ):

        passed = False

    # --------------------------------------------------------
    # INVALID COMMANDS
    # --------------------------------------------------------

    print()
    print("3. Testing invalid commands...")

    invalid_commands = (
        "",
        "hello",
        "restart",
        "logs",
        "logs-on-now",
        "add-wifi",
        "add-wifi|",
        "add-wifi||password"
    )

    for text in invalid_commands:

        if not _test_invalid(
            text
        ):

            passed = False

    # --------------------------------------------------------
    # QUEUE
    # --------------------------------------------------------

    print()
    print("4. Testing command queue...")

    clear()

    if not receive(
        "status",
        log=lambda message: None
    ):

        print(
            "FAILED: Could not queue status."
        )

        passed = False

    elif not has_pending_command():

        print(
            "FAILED: Queue reports no pending command."
        )

        passed = False

    elif pending_command().get(
        "command"
    ) != COMMAND_STATUS:

        print(
            "FAILED: Wrong command in queue."
        )

        passed = False

    else:

        print(
            "OK: Command successfully queued."
        )

    # --------------------------------------------------------
    # PREVENT OVERWRITE
    # --------------------------------------------------------

    print()
    print(
        "5. Testing pending-command protection..."
    )

    second_accepted = receive(
        "reboot",
        log=lambda message: None
    )

    if second_accepted:

        print(
            "FAILED: Second command overwrote pending command."
        )

        passed = False

    elif pending_command().get(
        "command"
    ) != COMMAND_STATUS:

        print(
            "FAILED: Pending command changed."
        )

        passed = False

    else:

        print(
            "OK: Pending command was protected."
        )

    # --------------------------------------------------------
    # TAKE COMMAND
    # --------------------------------------------------------

    print()
    print("6. Testing take_command()...")

    taken = take_command()

    if (
        taken is None
        or
        taken.get("command") != COMMAND_STATUS
    ):

        print(
            "FAILED: Wrong command returned."
        )

        passed = False

    elif has_pending_command():

        print(
            "FAILED: Queue was not cleared."
        )

        passed = False

    else:

        print(
            "OK: Command returned and queue cleared."
        )

    # --------------------------------------------------------
    # STOP CLASSIFICATION
    # --------------------------------------------------------

    print()
    print(
        "7. Testing chair-stop classification..."
    )

    stop_commands = (
        COMMAND_UPDATE,
        COMMAND_ROLLBACK,
        COMMAND_REBOOT
    )

    non_stop_commands = (
        COMMAND_STATUS,
        COMMAND_CHECK_UPDATE,
        COMMAND_ADD_WIFI,
        COMMAND_LOGS_ON,
        COMMAND_LOGS_OFF
    )

    for command_name in stop_commands:

        command = {
            "command": command_name
        }

        if not requires_chair_stop(
            command
        ):

            print(
                "FAILED: {} should require chair stop.".format(
                    command_name
                )
            )

            passed = False

        else:

            print(
                "OK: {} requires chair stop.".format(
                    command_name
                )
            )

    for command_name in non_stop_commands:

        command = {
            "command": command_name
        }

        if requires_chair_stop(
            command
        ):

            print(
                "FAILED: {} should not require chair stop.".format(
                    command_name
                )
            )

            passed = False

        else:

            print(
                "OK: {} does not require chair stop.".format(
                    command_name
                )
            )

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------

    clear()

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    print()

    if passed:

        print("==============================")
        print("COMMAND SELF-TEST PASSED")
        print("==============================")

    else:

        print("==============================")
        print("COMMAND SELF-TEST FAILED")
        print("==============================")

    print()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    self_test()