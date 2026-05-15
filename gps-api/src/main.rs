use std::{collections::HashMap, ops::Deref, sync::Mutex};

use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use warp::{
    Filter,
    filters::body,
    reject::{self, Rejection},
    reply::{self, Html, Reply, Response},
};

lazy_static! {
    static ref last_data: Mutex<GPSdata> = Mutex::new(GPSdata::disconnected());
}

#[derive(Serialize, Deserialize, Debug, Clone)]
enum FixStatus {
    Fix,
    Waiting,
    Disconnected,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct GPSdata {
    status: FixStatus,

    lat: f32,
    lon: f32,

    alt: f32,

    heading: f32,
}

impl GPSdata {
    fn disconnected() -> GPSdata {
        GPSdata {
            status: FixStatus::Disconnected,
            lat: 0.,
            lon: 0.,
            alt: 0.,
            heading: 0.,
        }
    }
    fn validate(&self) -> Result<(), GPSInvariantFail> {
        if self.lat < -90. || self.lat > 90. {
            return Err(GPSInvariantFail::BadLatitude(self.lat));
        }

        if self.lon < -180. || self.lon > 180. {
            return Err(GPSInvariantFail::BadLongitude(self.lon));
        }

        if self.heading < 0. || self.heading > 360. {
            return Err(GPSInvariantFail::BadHeading(self.heading));
        }
        Ok(())
    }
}

#[derive(Debug)]
enum GPSInvariantFail {
    BadLatitude(f32),
    BadLongitude(f32),
    BadHeading(f32),
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ErrorMessage {
    code: u16,
    message: String,
}

impl Reply for GPSInvariantFail {
    fn into_response(self) -> Response {
        let code = warp::http::StatusCode::BAD_REQUEST;

        let body = match self {
            GPSInvariantFail::BadLatitude(lat) => {
                format!("latitude is out of bounds (-90-90): {lat}")
            }
            GPSInvariantFail::BadLongitude(lon) => {
                format!("Longitude is out of bounds (-180-180): {lon}")
            }
            GPSInvariantFail::BadHeading(h) => format!("heading is out of bounds (0-360): {h}"),
        };

        let json = warp::reply::json(&ErrorMessage {
            code: code.as_u16(),
            message: body,
        });
        warp::reply::with_status(json, code).into_response()
    }
}

fn new_gps(json: GPSdata) -> Result<Response, GPSInvariantFail> {
    if let Err(v) = json.validate() {
        return Err(v);
    }
    println!("New gps frame: {:?}", json);

    let mut lock = last_data.lock().expect("The lock shopuldn't be poisoned.");
    *lock = json;
    Ok("".into_response())
}
fn get_gps() -> Response {
    let value: GPSdata = last_data
        .lock()
        .expect("The lock shouldn't be poisoned.")
        .clone();
    reply::json(&value).into_response()
}

#[tokio::main]
async fn main() {
    let post = warp::path("gps")
        .and(warp::post())
        .and(body::json())
        .map(new_gps);
    let get = warp::path("gps").and(warp::get()).map(get_gps);

    println!("Hello, world!");

    let routes = get.or(post);

    warp::serve(routes).run(([0, 0, 0, 0], 8880)).await;
    println!("Hosting on 0.0.0.0:8880")
}
