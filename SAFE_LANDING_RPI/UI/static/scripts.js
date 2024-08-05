let updateBoxTimer;
let originalWidth, originalHeight, displayedWidth, displayedHeight;


function safeLanding() {
    fetch('/safe_landing', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ action: 'safe_landing' })
    })
    .then(response => response.json())
    .then(data => {
        console.log(data.message);
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

function updateBoxPosition(x, y) {
    var img = document.getElementById("video");
    var box = document.getElementById("overlay-box");

    var scaledX = x * (displayedWidth / originalWidth);
    var scaledY = y * (displayedHeight / originalHeight);

    box.style.left = scaledX + "px";
    box.style.top = scaledY + "px";
    console.log(`Scaled box coordinates: (${scaledX}, ${scaledY})`);
    box.style.display = "block";
}

function hideBox() {
    var box = document.getElementById("overlay-box");
    box.style.display = "none";
}

function fetchBoxCoordinates() {
    fetch('/box_coords')
        .then(response => response.json())
        .then(data => {
            if (data.x && data.y) {
                updateBoxPosition(data.x, data.y);
            } else {
                hideBox();
            }
        })
        .catch(error => {
            console.error('Error fetching box coordinates:', error);
            hideBox();
        });
}

function startUpdateTimer() {
    updateBoxTimer = setInterval(fetchBoxCoordinates, 100);
}

function stopUpdateTimer() {
    clearInterval(updateBoxTimer);
}

window.onload = function() {
    var img = document.getElementById("video");
    originalWidth = img.naturalWidth;
    originalHeight = img.naturalHeight;
    displayedWidth = img.clientWidth;
    displayedHeight = img.clientHeight;
    
    startUpdateTimer();
}

window.onunload = function() {
    stopUpdateTimer();
}

