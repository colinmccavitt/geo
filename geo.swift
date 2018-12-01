//
//  geo.swift
//  perdix
//
//  Created by Colin McCavitt on 11/9/18.
//  Copyright © 2018 Colin McCavitt. All rights reserved.
//

import Foundation
import CoreLocation

struct LLA {
    var latitude : Double = 0.0
    var longitude : Double = 0.0
    var altitudeFeet : Double = 0.0
    
    init (latitude: Double, longitude: Double, altitudeFeet: Double){
        
        self.latitude = latitude
        self.longitude = longitude
        self.altitudeFeet = altitudeFeet
    }
    
    func coordinate() -> CLLocationCoordinate2D {
        
        return CLLocationCoordinate2DMake(self.latitude, self.longitude)
    }
    
    func location() -> CLLocation {
        
        return CLLocation(latitude: self.latitude, longitude: self.longitude)
        
    }
    
    func location2D() -> CLLocationCoordinate2D {
        
        return CLLocationCoordinate2D(latitude: self.latitude, longitude: self.longitude)
    }
    
    func altitudeMeters() -> Int {
        
        return Int(feet2meters(feet: self.altitudeFeet))
    }
    
}

struct Waypoint {
    
    var waypointID : String
    var waypointName : String
    var location : LLA
    
    init(waypointID: String, waypointName: String, location: LLA){
    
        self.waypointID = waypointID
        self.waypointName = waypointName
        self.location = location
    }
}

struct WaypointList {
    
    var waypoints: [Waypoint] = []
    
    mutating func loadWaypoints(waypoints: [Waypoint]) -> Void {
        
        self.waypoints = waypoints
        
    }
    
    init(waypoints: [Waypoint]){
        
        self.waypoints = waypoints
        
    }
    
}

struct AircraftFlightParameters {
    
    var ascentRateFps : Double = 0
    var descentRateFps : Double = 0
    var velocityFps : Double = 0
    var heading : Double?
    var currentLocation: LLA?
    var lastLocation : LLA?
    var lastSampleTime : Date?
    var lastWindMap = WindMap(windRecords: [])
    var maxAltitudeFeet = 80000
    var waypointList = WaypointList(waypoints: [])
    
    func elapsedTimeSeconds() -> Double {
        
        return abs(Date().secondsSince(lastSampleTime!))
        
    }
 
    mutating func updateLocation(multiplier: Double) -> LLA? {
        
        var newLocation:  LLA?
        
        let currentSampleTime = Date()
        let elapsedTimeSeconds = currentSampleTime.secondsSince(self.lastSampleTime!) * multiplier
        
        let climbRateFeet = Double((self.ascentRateFps * elapsedTimeSeconds) + (self.descentRateFps * elapsedTimeSeconds))
        
        var newAltitudeMeters = (currentLocation?.altitudeMeters())! + Int(feet2meters(feet: climbRateFeet))
        
        if newAltitudeMeters > Int(feet2meters(feet: Double(self.maxAltitudeFeet))) {
            
            self.ascentRateFps = 0
        }
        
        if let windRecord = self.lastWindMap.windsAt(altitudeMeters: newAltitudeMeters) {
        
            newLocation = futureLocation(fromLocation: lastLocation!, speedMPH: knots2mph(knots: Double(windRecord.windSpeedKnots)), timeSeconds: elapsedTimeSeconds, heading: Double(windRecord.windDirection))
        
            self.currentLocation = newLocation
            self.lastLocation = currentLocation
            self.lastSampleTime = currentSampleTime
            self.currentLocation?.altitudeFeet = meters2feet(meters: Double(newAltitudeMeters))
        }
        
        return newLocation
        
        
    }
}

struct AutoPilotParameters{
    var maxSpeedMPH : Int = 5000
    var maxTurnRateDPS : Double = 50.0
    var maxClimbFPS : Int = 3000
    var maxAccellerationRateFPS : Double = 200.0
    
    var currentAccellerationRateFPS: Double = 0.0
    var currentSpeedFPS : Double = 0.0
    var currentTurnRateDPS : Double = 0.0
    var currentHeadingDegrees : Double = 0.0
    
    var currentWaypoint: LLA?
    var previousWaypoint: LLA?
    var nextWaypoint: LLA?

}

/*(lat1:Double, lon1:Double, lat2:Double, lon2:Double, unit:String) -> Double {
    let theta = lon1 - lon2
    var dist = sin(deg2rad(deg: lat1)) * sin(deg2rad(deg: lat2)) + cos(deg2rad(deg: lat1)) * cos(deg2rad(deg: lat2)) * cos(deg2rad(deg: theta))
    dist = acos(dist)
    dist = rad2deg(rad: dist)
    dist = dist * 60 * 1.1515
    if (unit == "K") {
        dist = dist * 1.609344
    }
    else if (unit == "N") {
        dist = dist * 0.8684
    }
    return dist
}*/

/*func bearing(fromLocation: CLLocation, toLocation: CLLocation) -> Double {
    
    let fLat = deg2rad(deg: fromLocation.coordinate.latitude)
    let fLng = deg2rad(deg: fromLocation.coordinate.longitude)
    let tLat = deg2rad(deg: toLocation.coordinate.latitude)
    let tLng = deg2rad(deg: toLocation.coordinate.longitude)
    
    let degree = rad2deg(rad: atan2(sin(tLng-fLng)*cos(tLat), cos(fLat)*sin(tLat)-sin(fLat)*cos(tLat)*cos(tLng-fLng)))
    
    if (degree >= 0) {
        return degree;
    } else {
        return 360+degree;
    }
}*/


func midPoint(startPoint: LLA, endPoint: LLA) -> LLA{
    
    let φ1 = deg2rad(deg: startPoint.latitude)
    let λ1 = deg2rad(deg:startPoint.longitude)
    
    let φ2 = deg2rad(deg: endPoint.latitude)
    let λ2 = deg2rad(deg:endPoint.longitude)
    
    let Δλ =  λ2 - λ1 
    
    let Bx = cos(φ2) * cos(Δλ)
    let By = cos(φ2) * sin(Δλ)
    
    let φm = atan2( sin(φ1) + sin(φ2), sqrt(square(value: cos(φ1) + Bx) + square(value: By) ))
    
    let λm = λ1 + atan2(By, cos(φ1)+Bx)
    
    let newLocation = LLA(latitude: rad2deg(rad: φm), longitude: rad2deg(rad: λm), altitudeFeet: 0)
    
    return newLocation

}

func intermediatePoint(startPoint: LLA, endPoint: LLA, percentageOfLine: Double) -> LLA {
    
    let δ = distanceKM(startPoint: startPoint, endPoint: endPoint) / 6371.0

    let f = percentageOfLine / 100.0
    
    let φ1 = deg2rad(deg: startPoint.latitude)
    let λ1 = deg2rad(deg:startPoint.longitude)
    
    let φ2 = deg2rad(deg: endPoint.latitude)
    let λ2 = deg2rad(deg:endPoint.longitude)
    
    
    let a = sin((1 - f) * δ) / sin(δ)
    let b = sin(f * δ) / sin(δ)
    
    let  x = a * cos(φ1) * cos(λ1) + b * cos(φ2) * cos(λ2)
    let  y = a * cos(φ1) * sin(λ1) + b * cos(φ2) * sin(λ2)
    let  z = a * sin(φ1) + b * sin(φ2)
    
    let φi = atan2(z, sqrt(square(value: x) + square(value: y)))
    
    let λi = atan2(y, x)
    
    let newLocation = LLA(latitude: rad2deg(rad: φi), longitude: rad2deg(rad: λi), altitudeFeet: 0)
    
    return newLocation
}

func intersection(firstPoint: LLA, firstBearing: Double, secondPoint: LLA, secondBearing:  Double) -> LLA {
    
    
    let φ1 = deg2rad(deg: firstPoint.latitude)
    let λ1 = deg2rad(deg:firstPoint.longitude)
    let θ13 = deg2rad(deg: firstBearing)
    
    let φ2 = deg2rad(deg: secondPoint.latitude)
    let λ2 = deg2rad(deg:secondPoint.longitude)
    let θ23 = deg2rad(deg: secondBearing)
    
    let Δφ = φ2 - φ1
    let Δλ = λ2 - λ1
    
    let δ12 = 2 * asin( sqrt(square(value: sin(Δφ/2)) + cos(φ1) * cos(φ2) * square(value: sin(Δλ/2)) ))    //angular dist. p1–p2
    
    let θa = acos( ( sin(φ2) - sin(φ1) * cos(δ12) ) / ( sin(δ12) * cos(φ1) ) )
    
    let θb = acos( ( sin(φ1) - sin(φ2) * cos(δ12) ) / ( sin(δ12) * cos(φ2) ) )
    
    var θ12 = 0.0
    var θ21 = 0.0
    
    if sin(λ2 - λ1) > 0 {
        
        θ12 = θa
        θ21 = 2 * Double.pi - θb
        
    } else {
        
        θ12 = 2 * Double.pi - θa
        θ21 = θb
        
    }
    
    let α1 = θ13 - θ12
    let α2 = θ21 - θ23
    
    let α3 = acos( -cos( α1) * cos(α2) + sin(α1) * sin(α2) * cos(δ12) )
    
    let δ13 = atan2( sin(δ12) * sin(α1) * sin(α2) , cos(α2) + cos(α1) * cos(α3) )
    
    let φ3 = asin( sin(φ1) * cos(δ13) + cos(φ1) * sin(δ13) * cos(θ13) )
    
    let Δλ13 = atan2( sin(θ13) * sin(δ13) * cos(φ1) , cos(δ13) - sin(φ1) * sin(φ3) )
    
    let λ3 = λ1 + Δλ13
    
    let newLocation = LLA(latitude: rad2deg(rad: φ3), longitude: rad2deg(rad: λ3), altitudeFeet: 0)
    
    return newLocation
    
    /*
 
     
     α1 = θ13 − θ12
     α2 = θ21 − θ23    angle p2–p1–p3
     angle p1–p2–p3
     α3 = acos( −cos α1 ⋅ cos α2 + sin α1 ⋅ sin α2 ⋅ cos δ12 )    angle p1–p2–p3
     δ13 = atan2( sin δ12 ⋅ sin α1 ⋅ sin α2 , cos α2 + cos α1 ⋅ cos α3 )    angular dist. p1–p3
     φ3 = asin( sin φ1 ⋅ cos δ13 + cos φ1 ⋅ sin δ13 ⋅ cos θ13 )    p3 lat
     Δλ13 = atan2( sin θ13 ⋅ sin δ13 ⋅ cos φ1 , cos δ13 − sin φ1 ⋅ sin φ3 )    long p1–p3
     λ3 = λ1 + Δλ13
     
 */
    
    
}


func bearing(startPoint: LLA, endPoint: LLA) -> Double {
    
    let φ1 = deg2rad(deg: startPoint.latitude)
    let λ1 = deg2rad(deg:startPoint.longitude)
    
    let φ2 = deg2rad(deg: endPoint.latitude)
    let λ2 = deg2rad(deg:endPoint.longitude)
    
    //let Δφ = φ2 - φ1
    let Δλ = λ2 - λ1
    
    //let R = 6371.0
    
    
    let θ = atan2( sin(Δλ) * cos(φ2) , cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ) )
    
    return rad2deg(rad: θ)
    
}


func distanceKM(startPoint: LLA, endPoint: LLA) -> Double {
    
    let φ1 = deg2rad(deg: startPoint.latitude)
    let λ1 = deg2rad(deg:startPoint.longitude)
    
    let φ2 = deg2rad(deg: endPoint.latitude)
    let λ2 = deg2rad(deg:endPoint.longitude)
    
    let Δφ = φ2 - φ1
    let Δλ = λ2 - λ1
    
    let R = 6371.0
    
    let a = square(value: sin(Δφ/2)) + cos(φ1) * cos(φ2) * square(value: sin(Δλ/2))
    let c = 2 * atan2( sqrt(a), sqrt((1 - a)) )
    let d = R * c
    
    return d
}


func square(value: Double) -> Double {
    
    return value * value
    
}

func futureLocation(fromLocation: LLA, speedMPH: Double, timeSeconds: Double, heading: Double) -> LLA {

    let φ1 = deg2rad(deg: fromLocation.latitude)
    let λ1 = deg2rad(deg:fromLocation.longitude)
    
    //let θ = deg2rad(deg: heading)
    let θ = deg2rad(deg: heading)
    
    let kph = speedMPH * 1.60934
    
    let kps = kph * 0.000277778
    
    let d: Double = kps * timeSeconds
    let R = 6371.0 // Earths Radius in KM
    
    let δ = d/R
    
    let φ2 = asin( sin(φ1) * cos(δ) + cos(φ1) * sin(δ) * cos(θ) )
    let λ2 = λ1 + atan2( sin(θ) * sin(δ) * cos(φ1), cos(δ) - sin(φ1) * sin(φ2) )
    //let λ2 = λ1 + atan2( cos(δ) - sin(φ1) * sin(φ2) , sin(θ) * sin(δ) * cos(φ1) )
    
    let retLoc = LLA(latitude: rad2deg(rad: φ2), longitude: rad2deg(rad: λ2), altitudeFeet: 0)
    
    return retLoc
}

func deg2rad(deg:Double) -> Double {
    return deg * Double.pi / 180
}

func rad2deg(rad:Double) -> Double {
    return rad * 180.0 / Double.pi
}

func mph2fps(mph: Double) -> Double {
    return mph * 1.4667
}

func mph2mps(mph: Double) -> Double {
    return mph * 0.44704
}

func fps2mph(fps: Double) -> Double {
    return fps * 0.681818
}

func mps2fps(mps: Double) -> Double {
    return mps * 3.28084
}

func knots2fps(knots: Double) -> Double {
    return knots * 1.689
}

func knots2mph(knots: Double) -> Double {
    
    return knots * 1.15078
}

func knots2mps(knots: Double) -> Double {
    
    return knots * 0.514444
}

func km2miles(km: Double) -> Double {
    
    return km * 0.621371
}

func feet2miles(feet: Double) -> Double {
    return feet * 0.000189394
}

func feet2meters(feet: Double) -> Double {
    return feet * 0.3048
}

func miles2feet(miles: Double) -> Double {
    return miles/5280.0
}

func milesPerLonDeg(latDeg: Double) -> Double {
    return 69.170845 * cos(deg2rad(deg: latDeg))
}

func milesPerLatDeg(latDeg: Double) -> Double {
    if abs(latDeg) > 0.01 {
        return 68.708 + (69.386 - 68.708) * abs(latDeg) / 90
    } else {
        return 68.708
    }
}

func degPerLonMile(latDeg: Double) -> Double{
    return  1 / milesPerLonDeg(latDeg: latDeg)
}

func degPerLatMile(latDeg: Double) -> Double {
    return 1 / milesPerLatDeg(latDeg: latDeg)
}

func meters2miles(meters: Double) -> Double{
    return meters * 0.000621371
}

func mps2mph(mps: Double) -> Double{
    
    return mps * 2.23694
}

func meters2feet(meters: Double) -> Double {
    return meters * 3.28084
}

func mps2knots(mps: Double) -> Double {
    
    return mps * 1.94384
}

func calculateReciprical(heading: Double) -> Double{
  
  if heading < 180.0 {
    return heading + 180.0
  } else if heading == 180.0{
    return 360.0
  } else {
    return 180.0 - (360.0 - heading)
  }
  
}
