# Builds serial-native in a container, to compile serialcom for a different architecture
# Invoke using build-serialcom.sh
FROM eclipse-temurin:17

RUN apt-get update && apt-get install -y build-essential

# download gradle wrapper for docker to cache it
WORKDIR /serial-native
COPY *.gradle gradle.* gradlew ./
COPY gradle ./gradle
RUN ./gradlew --version

# actual build
COPY . .
RUN ./gradlew build
