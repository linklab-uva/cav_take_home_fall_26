#!/bin/bash
# Setup script to disable FastDDS SHM transport
# Run this once in Docker to create the config file

# Create the config file in /tmp (accessible from anywhere)
cat > /tmp/disable_shm_fastdds.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="no_shm_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML

echo "FastDDS SHM disable config created at /tmp/disable_shm_fastdds.xml"
echo ""
echo "To use it, export this in your terminal:"
echo "  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_shm_fastdds.xml"
echo ""
echo "Or add it to your ~/.bashrc for persistence:"
echo "  echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_shm_fastdds.xml' >> ~/.bashrc"
