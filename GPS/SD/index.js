var map = L.map('map'),
              realtime = L.realtime('status.json', {
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
            var theContent = '<table>';
            for (var i in feature.properties) {
                theContent += '<tr><td>' + i + '</td><td>'+ feature.properties[i] + '</td></tr>';
            }
            theContent += '</table>';
            return theContent;
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

$.getJSON( "status.json", function( data ) {
  var items = [];
  $.each( data, function( key, val ) {
    items.push( "<li id='" + key + "'>" + val + "</li>" );
  });

  $( "<ul/>", {
    "class": "my-new-list",
    html: items.join( "" )
  }).appendTo( "body" );
});
