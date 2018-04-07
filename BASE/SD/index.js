var map = L.map('map'),
              realtime = L.realtime('gps.json', {
              interval: 3 * 1000
              }).addTo(map);

L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: 'testing...'
}).addTo(map);

realtime.on('update', function(e) {

        popupContent = function(fId) {
            var feature = e.features[fId],
                c = feature.geometry.coordinates;
                p = feature.properties
                gpsTime = moment.unix(p.datetime)
            return '<strong>REMOTE :'+ p.remote_ID + '</strong></br>' +
                    'altitude :' + p.altitude+ 'm</BR>' +
                    'speed :' + p.speed+ 'km/h</BR>' +
                    'course :' + p.course+ 'deg</BR>' +
                    'battery :' + p.battery+ 'V</BR>' +
                    'RSSI :' + p.RSSI+ '</BR>' +
                    'datetime :' + p.datetime+ '</BR>'  +
                    'gpsTime :' + gpsTime.format('dddd, MMMM Do, YYYY h:mm:ss A');
        },
        bindFeaturePopup = function(fId) {
            realtime.getLayer(fId).bindPopup(popupContent(fId));
        },
        updateFeaturePopup = function(fId) {
            realtime.getLayer(fId).getPopup().setContent(popupContent(fId));
        };

    map.fitBounds(realtime.getBounds(), {});

    Object.keys(e.enter).forEach(bindFeaturePopup);
    Object.keys(e.update).forEach(updateFeaturePopup);
});
