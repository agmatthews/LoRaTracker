var map = L.map('map')
map.fitWorld();
map.setZoom(18);
var realtime = L.realtime('gps.json', {
                interval: 3 * 1000,
                pointToLayer: function (feature, latlng) {
                  if (feature.properties.type == 'base') {
                    return L.marker(latlng, {
                      icon: L.icon({
                        iconUrl: "icons/PILOT-blk.png",
                        iconSize: [24, 24],
                        iconAnchor: [12, 12],
                        popupAnchor: [0, -20]
                      }),
                      title: feature.properties.id,
                      riseOnHover: true
                    });
                  }
                  else if (feature.properties.type == 'drone') {
                    return L.marker(latlng, {
                      icon: L.icon({
                        iconUrl: "icons/DRONE-red.png",
                        iconSize: [24, 24],
                        iconAnchor: [12, 12],
                        popupAnchor: [0, -20]
                      }),
                      title: feature.properties.id,
                      riseOnHover: true
                    });
                  }
                  else {
                    return L.marker(latlng, {
                      icon: L.icon({
                        iconUrl: "icons/DRONE-gry.png",
                        iconSize: [24, 24],
                        iconAnchor: [12, 12],
                        popupAnchor: [0, -20]
                      }),
                      title: feature.properties.id,
                      riseOnHover: true
                    });
                  }
                }
            }).addTo(map);

// drone icon from
//https://www.flaticon.com/free-icon/drone_90904#term=drone&page=1&position=7

var baseLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}', { attribution: 'testing...' }).addTo(map);
// https://cartodb-basemaps-{s}.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png
// https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}
// https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}
// https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}

function styleLines(segments) {
    return {
                color: red,
                opacity: .7,
                dashArray: '20,15',
                lineJoin: 'round'
            };
}
realtime.on('update', function(e) {
        popupContent = function(fId) {
            var feature = e.features[fId];
            var c = feature.geometry.coordinates;
            var p = feature.properties;
            var popupcontent = [];
            if (feature.properties.type == 'drone') {
              map.panTo(new L.LatLng(c[1],c[0]));
              //console.log(c[1],c[0])
            }
            for (var prop in feature.properties) {
                popupcontent.push(prop + ": " + feature.properties[prop]);
            }
          return popupcontent.join("<br />");
        },
        bindFeaturePopup = function(fId) {
          realtime.getLayer(fId).bindPopup(popupContent(fId));
        },
        updateFeaturePopup = function(fId) {
          realtime.getLayer(fId).getPopup().setContent(popupContent(fId));
        };
        Object.keys(e.enter).forEach(bindFeaturePopup);
        Object.keys(e.update).forEach(updateFeaturePopup);
});

$("button").click(function(){
    startStatusUpdate()
});

function startStatusUpdate() {
    statusUpdate = setInterval(function () {
      $("#status").html("");
      $.getJSON("status.json", function(result){
          var content = '<table class="statusTable"><thead><tr><td>item</td><td>value</td></tr></thead>'
          $.each(result, function(i, field){
              //console.log (i + ':' + field)
              //$("#status").append('</br>'+ i + ':' + field );
              content += '<tr><td>' + i + '</td><td>' +  field + '</td></tr>';
          });
          content += "</table>"
          $('#status').append(content);
          $('.statusTable').ReStable({
            rowHeaders: true, // Table has row headers?
            maxWidth: 200, // Size to which the table become responsive
          });
      });
      $.getJSON("config.json", function(result){
        var content = '<A href="'+result.log_filename.substring(4)+'">Log File</A>'
        console.log(result)
        $('#status').append(content);
      });

    }, 1000);
}

startStatusUpdate();
