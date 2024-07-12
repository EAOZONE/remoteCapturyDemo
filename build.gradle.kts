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
    implementation("us.ihmc:remotecaptury-java:1.0.3")
    implementation("us.ihmc:psyonic-ability-hand-java:1.1.0")
}

tasks.test {
    useJUnitPlatform()
}