"""Calibrate and safely save the K230 red/green pillar LAB thresholds.

Competition-day use:
1. Keep the car still and run this file from RAM in CanMV.
2. Put the official red pillar inside RED and the official green pillar inside
   GREEN. Fill each box with a flat coloured face.
3. Keep the EMPTY MAT box clear and wait for the result.

A valid result is saved automatically to ``/sdcard/colorThresholds.cfg``.
The previous valid file is retained as ``/sdcard/colorThresholds.bak``. A bad
placement, invalid threshold, interrupted write, or failed read-back never
silently replaces the last known good calibration. This file controls no
actuators and sends no UART commands.
"""

import os


FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# Small sampling boxes make it easy to keep mat and pillar edges out of the
# measurement. Move the physical pillars until their flat coloured faces fill
# these boxes in the CanMV preview.
RED_ROI = (75, 85, 38, 72)
GREEN_ROI = (207, 85, 38, 72)
MAT_ROI = (145, 185, 30, 22)

ALIGN_SECONDS = 15
SAMPLE_FRAMES = 90
PREVIEW_DIVISOR = 3
# Allow for the observed exposure/lighting drift between repeated samples while
# keeping the red and green LAB-A bands on opposite sides of neutral.  These are
# margins around robust quartiles, not fixed copies of the rulebook RGB values.
L_MARGIN = 10
AB_MARGIN = 14

COLOR_CONFIG_LEGACY_MAGIC = "BUBBLEGUM_COLOR_V1"
COLOR_CONFIG_MAGIC = "BUBBLEGUM_COLOR_V2"
COLOR_CONFIG_PATH = "/sdcard/colorThresholds.cfg"
COLOR_CONFIG_BACKUP_PATH = "/sdcard/colorThresholds.bak"
COLOR_CONFIG_TEMP_PATH = "/sdcard/colorThresholds.tmp"
COLOR_CONFIG_MAX_BYTES = 320

# Require the complete threshold band to remain visibly chromatic on LAB A.
# These margins reject a neutral mat/white sample while retaining the latest
# field calibration (red A 24..47 and green A -33..-10).
MIN_RED_A_LOW = 8
MAX_GREEN_A_HIGH = -8


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def validate_lab_threshold(threshold):
    if not isinstance(threshold, (tuple, list)) or len(threshold) != 6:
        raise ValueError("LAB threshold must contain six integers")

    normalized = []
    for value in threshold:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("LAB threshold values must be integers")
        normalized.append(value)

    limits = ((0, 100), (0, 100)) + ((-128, 127),) * 4
    for value, bounds in zip(normalized, limits):
        if not bounds[0] <= value <= bounds[1]:
            raise ValueError("LAB threshold value outside sensor range")
    for low_index in (0, 2, 4):
        if normalized[low_index] >= normalized[low_index + 1]:
            raise ValueError("LAB threshold needs a non-zero range")
    return tuple(normalized)


def validate_color_threshold_pair(red_lab, green_lab):
    red_lab = validate_lab_threshold(red_lab)
    green_lab = validate_lab_threshold(green_lab)
    for threshold in (red_lab, green_lab):
        if threshold[1] - threshold[0] > 90:
            raise ValueError("LAB luminance threshold is implausibly wide")
        if threshold[3] - threshold[2] > 110:
            raise ValueError("LAB A threshold is implausibly wide")
        if threshold[5] - threshold[4] > 110:
            raise ValueError("LAB B threshold is implausibly wide")
    if red_lab[2] < MIN_RED_A_LOW:
        raise ValueError("red LAB A range is not clearly positive")
    if green_lab[3] > MAX_GREEN_A_HIGH:
        raise ValueError("green LAB A range is not clearly negative")
    if red_lab[2] <= green_lab[3]:
        raise ValueError("red and green LAB A thresholds overlap")
    return red_lab, green_lab


def thresholds_overlap(first, second):
    return all(
        not (first[index + 1] < second[index] or second[index + 1] < first[index])
        for index in (0, 2, 4)
    )


def validate_color_thresholds(red_lab, green_lab, magenta_lab=None):
    red_lab, green_lab = validate_color_threshold_pair(red_lab, green_lab)
    if magenta_lab is None:
        return red_lab, green_lab, None
    magenta_lab = validate_lab_threshold(magenta_lab)
    if magenta_lab[1] - magenta_lab[0] > 90:
        raise ValueError("magenta LAB luminance threshold is implausibly wide")
    if magenta_lab[3] - magenta_lab[2] > 110:
        raise ValueError("magenta LAB A threshold is implausibly wide")
    if magenta_lab[5] - magenta_lab[4] > 110:
        raise ValueError("magenta LAB B threshold is implausibly wide")
    if thresholds_overlap(red_lab, magenta_lab):
        raise ValueError("red and magenta LAB thresholds overlap")
    if thresholds_overlap(green_lab, magenta_lab):
        raise ValueError("green and magenta LAB thresholds overlap")
    return red_lab, green_lab, magenta_lab


def format_color_thresholds(red_lab, green_lab, magenta_lab=None):
    red_lab, green_lab, magenta_lab = validate_color_thresholds(
        red_lab, green_lab, magenta_lab
    )
    if magenta_lab is None:
        return "%s\nRED=%s\nGREEN=%s\n" % (
            COLOR_CONFIG_LEGACY_MAGIC,
            ",".join(str(value) for value in red_lab),
            ",".join(str(value) for value in green_lab),
        )
    return "%s\nRED=%s\nGREEN=%s\nMAGENTA=%s\n" % (
        COLOR_CONFIG_MAGIC,
        ",".join(str(value) for value in red_lab),
        ",".join(str(value) for value in green_lab),
        ",".join(str(value) for value in magenta_lab),
    )


def parse_color_thresholds(text):
    if not isinstance(text, str):
        raise ValueError("colour config must be text")
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines or lines[0] not in (
        COLOR_CONFIG_LEGACY_MAGIC,
        COLOR_CONFIG_MAGIC,
    ):
        raise ValueError("wrong colour config header or line count")
    expected_keys = (
        ("RED", "GREEN")
        if lines[0] == COLOR_CONFIG_LEGACY_MAGIC
        else ("RED", "GREEN", "MAGENTA")
    )
    if len(lines) != len(expected_keys) + 1:
        raise ValueError("wrong colour config header or line count")

    values = {}
    for line in lines[1:]:
        if "=" not in line:
            raise ValueError("malformed colour config line")
        key, encoded = line.split("=", 1)
        if key not in expected_keys or key in values:
            raise ValueError("unknown or duplicate colour config key")
        parts = encoded.split(",")
        if len(parts) != 6:
            raise ValueError("colour config threshold needs six values")
        try:
            threshold = tuple(int(part) for part in parts)
        except (TypeError, ValueError):
            raise ValueError("colour config contains a non-integer")
        values[key] = validate_lab_threshold(threshold)

    if any(key not in values for key in expected_keys):
        raise ValueError("colour config is missing a required threshold")
    return validate_color_thresholds(
        values["RED"], values["GREEN"], values.get("MAGENTA")
    )


def read_color_thresholds(path):
    with open(path, "r") as handle:
        text = handle.read(COLOR_CONFIG_MAX_BYTES + 1)
    if len(text) > COLOR_CONFIG_MAX_BYTES:
        raise ValueError("colour config is unexpectedly large")
    return parse_color_thresholds(text)


def remove_if_present(path):
    try:
        os.remove(path)
    except OSError:
        pass


def sync_storage():
    """Flush filesystem metadata when this CanMV firmware exposes os.sync."""

    sync = getattr(os, "sync", None)
    if sync is not None:
        sync()


def save_color_thresholds(
    red_lab,
    green_lab,
    magenta_lab=None,
    primary_path=COLOR_CONFIG_PATH,
    backup_path=COLOR_CONFIG_BACKUP_PATH,
    temp_path=COLOR_CONFIG_TEMP_PATH,
):
    """Install a verified config while keeping the previous good one usable."""

    # Validate before any filesystem mutation. A failed colour sample cannot
    # damage the configuration already on the card.
    red_lab, green_lab, magenta_lab = validate_color_thresholds(
        red_lab, green_lab, magenta_lab
    )
    payload = format_color_thresholds(red_lab, green_lab, magenta_lab)

    remove_if_present(temp_path)
    with open(temp_path, "w") as handle:
        handle.write(payload)
        handle.flush()
    sync_storage()

    # Never promote bytes that cannot be read and parsed exactly as written.
    expected = (red_lab, green_lab, magenta_lab)
    if read_color_thresholds(temp_path) != expected:
        remove_if_present(temp_path)
        raise OSError("temporary colour config failed read-back")

    previous_primary_is_valid = False
    try:
        read_color_thresholds(primary_path)
        previous_primary_is_valid = True
    except (OSError, ValueError):
        pass

    if previous_primary_is_valid:
        # The primary stays live until the verified temp file exists. If power
        # is lost after this rename, production still loads the backup.
        remove_if_present(backup_path)
        os.rename(primary_path, backup_path)
        sync_storage()
    else:
        # A corrupt primary must not block installation. An older valid backup,
        # if one exists, is deliberately left untouched.
        remove_if_present(primary_path)
        sync_storage()

    try:
        os.rename(temp_path, primary_path)
        sync_storage()
    except BaseException:
        remove_if_present(temp_path)
        raise

    # Verify the final name too. If storage corruption is detected, remove the
    # bad primary so production will fall back to the preserved backup.
    try:
        installed = read_color_thresholds(primary_path)
    except BaseException:
        remove_if_present(primary_path)
        raise
    if installed != expected:
        remove_if_present(primary_path)
        raise OSError("installed colour config failed read-back")
    return primary_path


def percentile(values, percent):
    ordered = sorted(values)
    index = ((len(ordered) - 1) * percent) // 100
    return ordered[index]


def new_samples():
    return {
        "l_low": [],
        "l_high": [],
        "a_low": [],
        "a_high": [],
        "b_low": [],
        "b_high": [],
        "l_mid": [],
        "a_mid": [],
        "b_mid": [],
    }


def add_sample(samples, stats):
    # Quartiles ignore small highlights and edge pixels better than raw min/max.
    samples["l_low"].append(stats.l_lq())
    samples["l_high"].append(stats.l_uq())
    samples["a_low"].append(stats.a_lq())
    samples["a_high"].append(stats.a_uq())
    samples["b_low"].append(stats.b_lq())
    samples["b_high"].append(stats.b_uq())
    samples["l_mid"].append(stats.l_median())
    samples["a_mid"].append(stats.a_median())
    samples["b_mid"].append(stats.b_median())


def recommended_threshold(samples):
    return (
        clamp(percentile(samples["l_low"], 10) - L_MARGIN, 0, 100),
        clamp(percentile(samples["l_high"], 90) + L_MARGIN, 0, 100),
        clamp(percentile(samples["a_low"], 10) - AB_MARGIN, -128, 127),
        clamp(percentile(samples["a_high"], 90) + AB_MARGIN, -128, 127),
        clamp(percentile(samples["b_low"], 10) - AB_MARGIN, -128, 127),
        clamp(percentile(samples["b_high"], 90) + AB_MARGIN, -128, 127),
    )


def median_colour(samples):
    return (
        percentile(samples["l_mid"], 50),
        percentile(samples["a_mid"], 50),
        percentile(samples["b_mid"], 50),
    )


def inside_threshold(colour, threshold):
    return (
        threshold[0] <= colour[0] <= threshold[1]
        and threshold[2] <= colour[1] <= threshold[3]
        and threshold[4] <= colour[2] <= threshold[5]
    )


def validate_pillar_sample(red_lab, green_lab, red_mid, green_mid, mat_mid):
    """Validate the two obstacle colours without requiring parking markers."""

    red_lab, green_lab = validate_color_threshold_pair(red_lab, green_lab)
    if red_mid[1] - green_mid[1] < 18:
        raise ValueError("red and green boxes are not clearly different")
    if inside_threshold(red_mid, green_lab):
        raise ValueError("GREEN threshold also matches the red sample")
    if inside_threshold(green_mid, red_lab):
        raise ValueError("RED threshold also matches the green sample")
    if inside_threshold(mat_mid, red_lab):
        raise ValueError("RED box included too much mat or shadow")
    if inside_threshold(mat_mid, green_lab):
        raise ValueError("GREEN box included too much mat or shadow")
    return red_lab, green_lab


def draw_guides(image, message):
    image.draw_rectangle(RED_ROI, color=(255, 0, 0), thickness=2)
    image.draw_rectangle(GREEN_ROI, color=(0, 255, 0), thickness=2)
    image.draw_rectangle(MAT_ROI, color=(255, 255, 255), thickness=1)
    image.draw_string_advanced(
        RED_ROI[0], RED_ROI[1] - 12, 10, "RED", color=(255, 0, 0)
    )
    image.draw_string_advanced(
        GREEN_ROI[0],
        GREEN_ROI[1] - 12,
        10,
        "GREEN",
        color=(0, 255, 0),
    )
    image.draw_string_advanced(
        MAT_ROI[0], MAT_ROI[1] - 10, 8, "EMPTY MAT", color=(255, 255, 255)
    )
    image.draw_string_advanced(5, 5, 10, message, color=(255, 255, 255))


def main():
    import time

    from media.display import Display
    from media.media import MediaManager
    from media.sensor import Sensor

    sensor = None
    sensor_running = False
    display_ready = False
    media_ready = False
    image = None

    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=30)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(
            Display.VIRT,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=10,
            to_ide=True,
            quality=70,
        )
        display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        sensor_running = True

        # Automatic exposure and white balance settle before alignment.
        for _ in range(30):
            sensor.snapshot()
            time.sleep_ms(20)

        print("BUBBLEGUM colour-only calibration started")
        print("RED in left box; GREEN in right box")
        print("Parking/magenta is disabled and is not required")
        print("Keep the EMPTY MAT box clear")

        align_start = time.ticks_ms()
        last_second = -1
        while time.ticks_diff(time.ticks_ms(), align_start) < ALIGN_SECONDS * 1000:
            os.exitpoint()
            image = sensor.snapshot()
            elapsed = time.ticks_diff(time.ticks_ms(), align_start) // 1000
            remaining = ALIGN_SECONDS - elapsed
            draw_guides(image, "ALIGN BLOCKS: %ds" % remaining)
            Display.show_image(image)
            if remaining != last_second:
                print("sampling in %d..." % remaining)
                last_second = remaining

        red_samples = new_samples()
        green_samples = new_samples()
        mat_samples = new_samples()

        for frame_number in range(SAMPLE_FRAMES):
            os.exitpoint()
            image = sensor.snapshot()

            # Measure before drawing so overlays cannot contaminate samples.
            add_sample(red_samples, image.get_statistics(roi=RED_ROI))
            add_sample(green_samples, image.get_statistics(roi=GREEN_ROI))
            add_sample(mat_samples, image.get_statistics(roi=MAT_ROI))

            if frame_number % PREVIEW_DIVISOR == 0:
                draw_guides(
                    image,
                    "SAMPLING %d/%d" % (frame_number + 1, SAMPLE_FRAMES),
                )
                Display.show_image(image)

        red_lab = recommended_threshold(red_samples)
        green_lab = recommended_threshold(green_samples)
        red_mid = median_colour(red_samples)
        green_mid = median_colour(green_samples)
        mat_mid = median_colour(mat_samples)

        # Parking is disabled: only official red, official green, and the empty
        # mat can decide whether this obstacle calibration is saved.
        try:
            red_lab, green_lab = validate_pillar_sample(
                red_lab,
                green_lab,
                red_mid,
                green_mid,
                mat_mid,
            )
            valid = True
        except ValueError as error:
            print("FAILED: unsafe colour thresholds:", error)
            valid = False

        print("red centre LAB:", red_mid)
        print("green centre LAB:", green_mid)
        print("empty mat centre LAB:", mat_mid)

        if valid:
            try:
                saved_path = save_color_thresholds(red_lab, green_lab)
                print("RED_LAB = %s" % (red_lab,))
                print("GREEN_LAB = %s" % (green_lab,))
                print("COLOR CALIBRATION SAVED:", saved_path)
                print("RED/GREEN SAVED; PARKING/MAGENTA REMAINS DISABLED")
                print("RESET K230 - production main.py will load this file")
            except BaseException as error:
                print("CALIBRATION NOT SAVED - storage verification failed:", error)
                print("Previous valid primary/backup remains usable")
        else:
            print("CALIBRATION NOT SAVED - move the pillars and run again")

    except KeyboardInterrupt:
        print("colour calibration stopped by user")
    except BaseException as error:
        if str(error) == "IDE interrupt":
            print("colour calibration stopped by IDE")
        else:
            print("colour calibration error:", error)
            raise
    finally:
        image = None
        if sensor_running:
            try:
                sensor.stop()
            except BaseException:
                pass
        if display_ready:
            try:
                Display.deinit()
            except BaseException:
                pass
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_ready:
            try:
                MediaManager.deinit()
            except BaseException:
                pass


if __name__ == "__main__":
    main()
