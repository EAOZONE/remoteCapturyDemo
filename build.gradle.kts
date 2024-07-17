plugins {
    id("java")
}

group = "us.ihmc"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
}

dependencies {
    testImplementation(platform("org.junit:junit-bom:5.10.0"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    implementation("us.ihmc:remotecaptury-java:1.0.4")
    implementation("us.ihmc:psyonic-ability-hand-java:1.1.1")
    implementation("us.ihmc:euclid:0.21.0")
    implementation("us.ihmc:euclid-frame:0.21.0")
    implementation("us.ihmc:ihmc-yovariables:0.12.0")
    implementation("us.ihmc:ihmc-robot-data-logger:0.29.0")
}

tasks.test {
    useJUnitPlatform()
}