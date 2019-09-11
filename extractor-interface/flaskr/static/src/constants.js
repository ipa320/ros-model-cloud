export const eventTypes = {
    SOCKET_ON_MESSAGE_LOG: 'SOCKET_ON_MESSAGE_LOG',
    SOCKET_ON_MESSAGE_MODELS: 'SOCKET_ON_MESSAGE_MODELS',
    SOCKET_ON_MESSAGE_ERRORS: 'SOCKET_ON_MESSAGE_ERRORS',
    SOCKET_ON_ERROR: 'SOCKET_ON_ERROR',
    SOCKET_ON_OPEN: 'SOCKET_ON_OPEN',
    SOCKET_ON_CLOSE:'SOCKET_ON_CLOSE',
    SOCKET_ON_MESSAGE_EXTRACTION_DONE: 'SOCKET_ON_MESSAGE_EXTRACTION_DONE',
    VALIDATION_ERROR: 'VALIDATION_ERROR',
    FILE_DOWNLOAD_ERROR: 'FILE_DOWNLOAD_ERROR'
};

export const errorMessages = {
    REPOSITORY_NOT_FOUND: (repositoryName) => `The repository "${repositoryName}" could not be found. Please ensure that the link is valid and the repository is public`,
    INVALID_FIELDS: () => 'Please fill out all fields.',
    NO_MODEL_GENERATED: () => 'There was a problem with the model generation.',
    SERVER_ERROR: () => 'An internal server error ocurred while processing the request.',
    LAUNCH_FILE_NOT_FOUND: (fileName) => `The launch file "${fileName}" could not be found in the repository`,
    FAILED_PACKAGES: (packages) => packages.length === 1 ?
    `The package '${packages[0]}' could not be built.` :
    `The packages ${packages.slice(0, packages.length - 1).map(p => `'${p}'`).join(', ')} and '${packages[packages.length - 1]}' could not be built.`
};