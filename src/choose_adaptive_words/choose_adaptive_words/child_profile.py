class ChildProfile:
    def __init__(self, profile, logger):
        self.first_name = profile["firstName"]
        self.last_name = profile["lastName"]
        self.birth_date = profile["birthDate"]
        self.gender = profile["gender"]
        self.handedness = profile["handedness"]

        logger.info("New Child Profile Created")
        logger.info("First Name: " + self.first_name)
        logger.info("Last Name: " + self.last_name)
        logger.info("Birth Date: " + self.birth_date)
        logger.info("Gender: " + self.gender)
        logger.info("Handedness: " + self.handedness)
