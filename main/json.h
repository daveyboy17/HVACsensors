

#define JSON_SENSOR_READING "{\"temperature\":%.1f,\"absolute\":%.0f,\"filter\":%.0f,\"blocked\":%d}"   // temp_C, absolute_hPa, delta_Pa, blocked_bool

/*
{
    "name":"%s",
    "unique_id":"%s",
    "state_topic":"%s",
    "unit_of_measurement":"%s"
    %s                                  // avaialability, device etc.
}
*/
#define JSON_DISCOVERY_CONFIG  "{\"name\":\"%s\",\"unique_id\":\"%s\",\"state_topic\":\"%s\",\"unit_of_measurement\":\"%s\"%s}"

/*
"device":
{
    "name":%s,
    "ids":%s,                           // device id (without the entity number suffix)
    "sw":%s,                            // software version, only needed in first entity
    "mf":DBTech                         // manufacturer, only needed in first entity
}
*/
#define JSON_DISCOVERY_DEVICE_CORE   ",\"device\":{\"name\":\"%s\",\"ids\":\"%s\",\"sw\":\"%s\"}"
#define JSON_DISCOVERY_DEVICE_ITEM   ",\"device\":{\"name\":\"%s\",\"ids\":\"%s\"}"

