import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import microphone, speaker
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_URL, CONF_TRIGGER_ID

CODEOWNERS = ["@maxmaxme"]
DEPENDENCIES = ["network", "microphone", "speaker"]

CONF_TOKEN = "token"
CONF_MICROPHONE = "microphone"
CONF_MIC_CHANNEL = "mic_channel"
CONF_INPUT_FORMAT = "input_format"
CONF_SPEAKER = "speaker"

# Maps to mic_mono16_: how on_mic_data_ should interpret incoming frames.
INPUT_FORMATS = {
    # XMOS path: interleaved stereo int32, one channel, audio in the high 16 bits.
    "stereo32": False,
    # Plain codec (e.g. ES8311): already int16 mono, forwarded as-is.
    "mono16": True,
}
CONF_DIAGNOSTICS = "diagnostics"
CONF_ON_PHASE = "on_phase"
CONF_ON_REPEATED_FAILURE = "on_repeated_failure"
CONF_ON_FOLLOWUP_OPENED = "on_followup_opened"

va_client_ns = cg.esphome_ns.namespace("va_client")
VaClient = va_client_ns.class_("VaClient", cg.Component)
OnPhaseTrigger = va_client_ns.class_(
    "OnPhaseTrigger", automation.Trigger.template(cg.std_string)
)
OnRepeatedFailureTrigger = va_client_ns.class_(
    "OnRepeatedFailureTrigger", automation.Trigger.template()
)
OnFollowupOpenedTrigger = va_client_ns.class_(
    "OnFollowupOpenedTrigger", automation.Trigger.template()
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(VaClient),
        cv.Required(CONF_URL): cv.string,
        cv.Required(CONF_TOKEN): cv.string,
        cv.Required(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
        cv.Optional(CONF_MIC_CHANNEL, default=0): cv.int_range(min=0, max=1),
        cv.Optional(CONF_INPUT_FORMAT, default="stereo32"): cv.enum(
            INPUT_FORMATS, lower=True
        ),
        cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
        # Per-turn latency anchors + WS gap / clipping / underrun detectors.
        # Opt-in because in steady-state production they just clutter the log
        # and burn a handful of cycles per audio frame on counters nothing
        # reads. Flip to true when chasing audio-quality bugs, then turn back
        # off after the fix lands.
        cv.Optional(CONF_DIAGNOSTICS, default=False): cv.boolean,
        cv.Optional(CONF_ON_PHASE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnPhaseTrigger),
            }
        ),
        cv.Optional(CONF_ON_REPEATED_FAILURE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnRepeatedFailureTrigger),
            }
        ),
        cv.Optional(CONF_ON_FOLLOWUP_OPENED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnFollowupOpenedTrigger),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # esp-idf managed component providing esp_websocket_client.
    esp32.add_idf_component(
        name="espressif/esp_websocket_client",
        ref="1.7.0",
    )
    # JSON parser for the control channel ({"type":"phase","value":"..."} etc.).
    # ArduinoJson is header-only and ESPHome already uses it in several core
    # components, so the build infra is well-trodden.
    cg.add_library("ArduinoJson", "7.4.2")

    if config[CONF_DIAGNOSTICS]:
        cg.add_define("USE_VA_CLIENT_DIAGNOSTICS")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_token(config[CONF_TOKEN]))
    cg.add(var.set_mic_channel(config[CONF_MIC_CHANNEL]))
    cg.add(var.set_mic_mono16(config[CONF_INPUT_FORMAT]))

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))

    spk = await cg.get_variable(config[CONF_SPEAKER])
    cg.add(var.set_speaker(spk))

    for conf in config.get(CONF_ON_PHASE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "phase")], conf)

    for conf in config.get(CONF_ON_REPEATED_FAILURE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_FOLLOWUP_OPENED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
